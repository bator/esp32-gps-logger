#!/usr/bin/env python3
"""CPython harness for gps_logger upload / crash / WiFi logic. Not deployed."""
from __future__ import print_function

import gc as real_gc
import importlib
import json
import os
import shutil
import sys
import tempfile
import time
import types

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

_clock = [100000]
_oled_fail = [False]
_resets = []
_posts = []
_post_code = [201]
_post_raise = [None]
_post_headers = [{}]
_wdt = []
_timers = []
_hostnames = []
_oled_inits = []
_reset_cause = [1]


class Pin(object):
    IN = 1
    PULL_UP = 2

    def __init__(self, n, *a, **k):
        self.n = n

    def value(self):
        return 1


class I2C(object):
    def __init__(self, *a, **k):
        pass


class UART(object):
    def __init__(self, *a, **k):
        pass

    def any(self):
        return 0

    def readinto(self, buf):
        return 0


class WDT(object):
    def __init__(self, timeout=5000):
        self.timeout = timeout
        self.fed = 0
        _wdt.append(self)

    def feed(self):
        self.fed += 1


class Timer(object):
    PERIODIC = 1
    ONE_SHOT = 0

    def __init__(self, tid=0):
        self.id = tid
        self.period = None
        self.callback = None
        _timers.append(self)

    def init(self, period=0, mode=0, callback=None):
        self.period = period
        self.mode = mode
        self.callback = callback
        self.deinited = False

    def deinit(self):
        self.deinited = True


def _reset():
    _resets.append(True)
    raise SystemExit("machine.reset")


class SSD1306_I2C(object):
    raise_show = False

    def __init__(self, w, h, i2c):
        _oled_inits.append((w, h))
        if _oled_fail[0]:
            raise OSError("no display")
        self.ops = []

    def fill(self, v):
        self.ops.append(("fill", v))

    def fill_rect(self, *a):
        self.ops.append(("fill_rect", a))

    def text(self, *a):
        self.ops.append(("text", a))

    def show(self):
        if SSD1306_I2C.raise_show:
            raise OSError("bus")
        self.ops.append(("show",))


class RTC(object):
    mem = b""

    def memory(self, data=None):
        if data is None:
            return RTC.mem
        RTC.mem = data if isinstance(data, bytes) else bytes(data)
        return None


class WLAN(object):
    """v1.29 esp32 STA mock (network_wlan.c status() / connect()).

    Numbers match harness STAT_* (IDLE=0 CONNECTING=1 GOT_IP=3
    WRONG_PASSWORD=4 BEACON_TIMEOUT=5). Beacon timeout while
    connect_requested reports CONNECTING; disconnect is ASSOC_LEAVE →
    IDLE; connect() does not clear the previous fail reason.
    """
    scan_aps = []
    scan_error = False
    ok = set()
    last_ssid = None
    connects = []
    delay = 0
    stale_fail = 0
    leftover = 4
    flaky = False
    connect_error = None
    recover_after = 0

    def __init__(self, iface):
        self._status = 0
        self._left = 0
        self._stale = 0
        self._want_ok = False
        self._requested = False
        self._active = False
        self.iface = iface
        WLAN.instances.append(self)

    def active(self, x=None):
        if x is None:
            return self._active
        self._active = bool(x)
        if not x:
            self._requested = False
            self._status = 0
        return None

    def scan(self):
        if WLAN.scan_error:
            raise OSError("scan fail")
        out = []
        for item in WLAN.scan_aps:
            ssid, rssi = item[0], item[1]
            bssid = item[2] if len(item) > 2 else (b"\x00" * 6)
            out.append((ssid.encode(), bssid, 1, rssi, 4, False))
        return out

    def connect(self, s, p, bssid=None):
        WLAN.last_ssid = s
        WLAN.connects.append((s, p, bssid))
        if WLAN.connect_error is True or s == WLAN.connect_error:
            raise OSError("connect")
        self._want_ok = s in WLAN.ok
        self._requested = True
        if self._want_ok:
            if WLAN.stale_fail:
                self._status = WLAN.leftover
                self._stale = WLAN.stale_fail
                self._left = WLAN.delay
            elif WLAN.delay:
                self._status = 1
                self._left = WLAN.delay
            else:
                self._status = 3
        else:
            if WLAN.stale_fail:
                self._status = WLAN.leftover
                self._stale = WLAN.stale_fail
                self._left = 0
            else:
                self._status = 4

    def disconnect(self):
        # WIFI_REASON_ASSOC_LEAVE → STAT_IDLE
        self._requested = False
        self._status = 0

    def status(self, arg=None):
        if arg == "rssi":
            for item in WLAN.scan_aps:
                if item[0] == WLAN.last_ssid:
                    return item[1]
            return -50
        if getattr(self, "_recover", 0) > 0 and self._status in (1, 5):
            self._recover -= 1
            if self._recover <= 0:
                self._status = 3
                self._requested = True
                return 3
            return 1
        if self._stale > 0:
            self._stale -= 1
            leftover = self._status
            if self._stale == 0:
                if self._want_ok:
                    if self._left > 0:
                        self._status = 1
                    else:
                        self._status = 3
                else:
                    self._status = 4
            return leftover
        if self._left > 0:
            self._left -= 1
            if self._left == 0 and self._status == 1:
                self._status = 3
            else:
                return 1
        if WLAN.flaky and self._status == 3:
            WLAN.flaky = False
            self._status = 1
            self._left = 1
            return 1
        if self._status == 5 and self._requested:
            return 1
        return self._status

    def ifconfig(self):
        return ("192.168.1.8", "255.255.255.0", "192.168.1.1", "1.1.1.1")

    def config(self, **k):
        pass


WLAN.instances = []


class FakeResp(object):
    def __init__(self, code, headers=None):
        self.status_code = code
        self.headers = dict(headers or {})
        self.closed = False

    def close(self):
        self.closed = True


def _post(url, data=None, headers=None, timeout=None):
    body = bytes(data) if data is not None else b""
    _posts.append({"url": url, "data": body, "headers": dict(headers or {}),
                   "timeout": timeout})
    if _post_raise[0]:
        err, _post_raise[0] = _post_raise[0], None
        raise err
    return FakeResp(_post_code[0], _post_headers[0])


def _hostname(name=None):
    if name is not None:
        _hostnames.append(name)
        return name
    return _hostnames[-1] if _hostnames else None


def _sleep_ms(ms):
    _clock[0] += int(ms)


def _install_mocks():
    machine = types.ModuleType("machine")
    machine.Pin = Pin
    machine.I2C = I2C
    machine.UART = UART
    machine.WDT = WDT
    machine.Timer = Timer
    machine.RTC = RTC
    machine.reset = _reset
    machine.WDT_RESET = 3
    machine.PWRON_RESET = 1
    machine.SOFT_RESET = 4
    machine.HARD_RESET = 2
    machine.reset_cause = lambda: _reset_cause[0]
    sys.modules["machine"] = machine

    ssd = types.ModuleType("ssd1306")
    ssd.SSD1306_I2C = SSD1306_I2C
    sys.modules["ssd1306"] = ssd

    net = types.ModuleType("network")
    net.STA_IF = 0
    net.STAT_IDLE = 0
    net.STAT_CONNECTING = 1
    net.STAT_GOT_IP = 3
    net.STAT_NO_AP_FOUND = 2
    net.STAT_WRONG_PASSWORD = 4
    net.STAT_BEACON_TIMEOUT = 5
    net.STAT_ASSOC_FAIL = 6
    net.STAT_HANDSHAKE_TIMEOUT = 7
    net.WLAN = WLAN
    net.hostname = _hostname
    sys.modules["network"] = net

    req = types.ModuleType("requests")
    req.post = _post
    sys.modules["requests"] = req

    fake_gc = types.ModuleType("gc")
    fake_gc.threshold = lambda *a, **k: None
    fake_gc.collect = lambda: real_gc.collect()
    fake_gc.mem_free = lambda: 123456
    sys.modules["gc"] = fake_gc

    time.ticks_ms = lambda: _clock[0]
    time.ticks_diff = lambda a, b: a - b
    time.ticks_add = lambda a, b: a + b
    time.sleep_ms = _sleep_ms
    time.sleep = lambda s: None


def _reload_logger():
    if "gps_logger" in sys.modules:
        return importlib.reload(sys.modules["gps_logger"])
    import gps_logger
    return gps_logger


_install_mocks()
sys.modules["machine"].WDT_RESET = 3
sys.modules["machine"].PWRON_RESET = 1
sys.modules["machine"].SOFT_RESET = 4
sys.modules["machine"].HARD_RESET = 2
sys.modules["machine"].reset_cause = lambda: _reset_cause[0]
gl = _reload_logger()

passed = 0
failed = 0
failures = []


def check(name, cond, detail=""):
    global passed, failed
    if cond:
        passed += 1
        print("PASS  %s" % name)
    else:
        failed += 1
        msg = "FAIL  %s%s" % (name, (" - " + detail) if detail else "")
        print(msg)
        failures.append(msg)


def make_gpx(path, n, extra=b"", times=None, extras=None):
    name = os.path.basename(path)[:-4]
    parts = [(gl.HEADER % name).encode()]
    for i in range(n):
        lat = 52.5 + i * 0.0001
        lon = 13.4 + i * 0.0001
        t = (times[i] if times else
             "2026-10-05T%02d:%02d:%02dZ" % (i // 3600, (i // 60) % 60, i % 60))
        extra_tags = extras[i] if extras else "<ele>%.1f</ele>" % 10.0
        parts.append(
            ('<trkpt lat="%.6f" lon="%.6f">%s'
             '<time>%s</time></trkpt>\n' % (lat, lon, extra_tags, t)).encode())
    if extra:
        parts.append(extra)
    parts.append(gl.FOOTER.encode())
    with open(path, "wb") as f:
        f.write(b"".join(parts))


def write_env(tmpdir, text):
    p = os.path.join(tmpdir, "env")
    with open(p, "w") as f:
        f.write(text)
    gl.ENV_FILE = p
    return p


def reset_wlan():
    WLAN.scan_aps = []
    WLAN.scan_error = False
    WLAN.ok = set()
    WLAN.last_ssid = None
    WLAN.connects = []
    WLAN.delay = 0
    WLAN.stale_fail = 0
    WLAN.leftover = 4
    WLAN.flaky = False
    WLAN.connect_error = None
    WLAN.recover_after = 0
    WLAN.instances = []


def online(tmpdir):
    gl.LOG_DIR = tmpdir
    try:
        for n in os.listdir(tmpdir):
            if n.endswith((".gpx", ".pos", ".tmp", ".bad")):
                try:
                    os.remove(os.path.join(tmpdir, n))
                except OSError:
                    pass
    except OSError:
        pass
    gl.log_reset()
    gl.wifi_reset()
    gl.up_reset()
    gl.wifi["state"] = "connected"
    gl.wifi["ssid"] = "home"
    gl.up["url"] = "https://dawarich.example.com"
    gl.up["key"] = "secret-key"
    gl.up["state"] = "idle"
    gl.up["next_at"] = _clock[0]
    gl.up["backoff"] = gl.BACKOFF_START
    gl.up["allow_bad"] = True
    gl.up["bad_tries"] = 0
    gl.log["file"] = None
    gl.log["online"] = True
    _posts[:] = []
    _post_code[0] = 201
    _post_raise[0] = None
    _post_headers[0] = {}


def pump():
    gl.up["next_at"] = _clock[0]
    gl.upload_tick()


def locations():
    out = []
    for p in _posts:
        out.extend(json.loads(p["data"].decode())["locations"])
    return out


# --- headless display -------------------------------------------------------

_oled_fail[0] = True
gl = _reload_logger()
check("headless: oled is None after SSD1306 init failure", gl.oled is None)
try:
    gl.draw()
    gl._oled_banner("hi")
    check("headless: draw() and banner are no-ops", True)
except Exception as e:
    check("headless: draw() and banner are no-ops", False, repr(e))
_oled_fail[0] = False
gl = _reload_logger()
check("oled present after reload with working display", gl.oled is not None)
gl.st.update(spin=0, rx=None, valid=False, fixtype=None, quality=None)
gl.wifi.update(state="off", ssid=None)
gl.log.update(online=False)
gl.up.update(left=0)
try:
    gl.draw()
    check("draw() talks to OLED when present",
          any(op[0] == "show" for op in gl.oled.ops))
except Exception as e:
    check("draw() talks to OLED when present", False, repr(e))

check("default DEVICE_ID is Dawarich-GPS-tracker",
      gl.DEVICE_ID == "Dawarich-GPS-tracker")
check("default HOSTNAME is Dawarich-GPS-tracker",
      gl.HOSTNAME == "Dawarich-GPS-tracker")

# Firmware-faithful WLAN (v1.29 network_wlan.c status / disconnect)
fw = WLAN(0)
fw.active(True)
fw.connect("x", "y")
fw._status = 5
fw._requested = True
check("FwWLAN: beacon timeout while connect_requested is CONNECTING",
      fw.status() == 1)
fw.disconnect()
check("FwWLAN: disconnect ASSOC_LEAVE is IDLE",
      fw.status() == 0 and fw._requested is False)


# --- GPX parse helpers ------------------------------------------------------

check("_attr missing closer returns None",
      gl._attr('<trkpt lat="52.5', "lat") is None)
check("_tag missing closer returns None",
      gl._tag("<trkpt><time>2026-10-05T10:00:00Z", "time") is None)
check("_attr complete value",
      gl._attr('<trkpt lat="52.5" lon="13.4">', "lat") == "52.5")
check("valid ISO time accepted",
      gl._valid_time("2026-10-05T12:00:00Z"))
check("quoted / short time rejected",
      not gl._valid_time('2026-10-05T12:00:00Z"')
      and not gl._valid_time("2026-10-05"))
check("inf ele is not finite",
      not gl._finite(float("inf")))


tmpdir = tempfile.mkdtemp(prefix="gpxlog-")
try:
    # --- truncated last <trkpt> --------------------------------------------
    online(tmpdir)
    path = os.path.join(tmpdir, "20261005_100000.gpx")
    make_gpx(path, 3, extra=b'<trkpt lat="52.9" lon="13.9"><time>2026-10-05T11:00:00Z')
    n, n_ok, size, new_off, eof = gl.read_batch(path, 0)
    check("truncated: consumed 4 trkpt lines (3 good + 1 truncated)", n == 4)
    check("truncated: encoded only 3 valid points", n_ok == 3)
    check("truncated: produced a JSON body", size > 0)
    body = json.loads(bytes(gl._body[:size]).decode())
    check("truncated: JSON has 3 locations", len(body["locations"]) == 3)
    check("truncated: default device_id in JSON",
          body["locations"][0]["properties"]["device_id"]
          == "Dawarich-GPS-tracker")

    pump()
    check("truncated: one POST then file deleted (n < UPLOAD_BATCH)",
          len(_posts) == 1 and not os.path.exists(path))
    check("truncated: no leftover .pos",
          not os.path.exists(path + ".pos"))
    check("truncated: Bearer header, no api_key query",
          _posts[0]["headers"].get("Authorization") == "Bearer secret-key"
          and "api_key=" not in _posts[0]["url"]
          and _posts[0]["url"].endswith("/api/v1/overland/batches"))

    # --- file > 330 points -------------------------------------------------
    online(tmpdir)
    big = os.path.join(tmpdir, "20261005_110000.gpx")
    make_gpx(big, 350)
    check("large: count_points from offset 0 is 350", gl.count_points() == 350)
    batches = 0
    while os.path.exists(big) and batches < 20:
        pump()
        batches += 1
    check("large: 7 POSTs for 350 points (50/batch)",
          len(_posts) == 7, "batches=%s posts=%s" % (batches, len(_posts)))
    check("large: all 350 points uploaded",
          len(locations()) == 350)
    check("large: GPX deleted after last offset", not os.path.exists(big))
    check("large: sidecar deleted with the file",
          not os.path.exists(big + ".pos"))
    check("pending() does not list .pos files",
          gl.pending() == [])

    ghost = os.path.join(tmpdir, "20261005_120000.gpx.pos")
    with open(ghost, "w") as f:
        f.write("12\n")
    check("pending() ignores a planted .pos sidecar",
          ghost not in gl.pending() and all(not p.endswith(".pos") for p in gl.pending()))
    os.remove(ghost)

    # --- interrupted POST / offset commit (at-least-once, no loss) ---------
    online(tmpdir)
    path = os.path.join(tmpdir, "20261005_130000.gpx")
    make_gpx(path, 80)
    real_write = gl.write_offset
    crash = [True]

    def crash_write(p, off):
        if crash[0]:
            crash[0] = False
            raise OSError("power loss")
        return real_write(p, off)

    gl.write_offset = crash_write
    pump()
    gl.write_offset = real_write
    check("crash: POST happened before offset commit failed",
          len(_posts) == 1)
    check("crash: no .pos sidecar committed",
          not os.path.exists(path + ".pos"))
    check("crash: GPX still has all 80 points",
          gl.count_points() == 80)
    first_body = _posts[0]["data"]
    pump()
    check("crash: retry POSTs the same first batch (at-least-once)",
          len(_posts) == 2 and _posts[1]["data"] == first_body)
    check("crash: offset committed on retry",
          os.path.exists(path + ".pos") or not os.path.exists(path))
    while os.path.exists(path):
        pump()
        if len(_posts) > 10:
            break
    check("crash: file eventually uploaded and deleted (no infinite loop)",
          not os.path.exists(path) and len(locations()) == 80 + 50,
          "posts=%s exists=%s" % (len(_posts), os.path.exists(path)))
    ts = [loc["properties"]["timestamp"] for loc in locations()]
    check("crash: 80 unique points kept (duplicate is the resent batch)",
          len(set(ts)) == 80)

    # --- power-loss model: leftover .pos.tmp, no .pos ----------------------
    online(tmpdir)
    path = os.path.join(tmpdir, "20261005_131000.gpx")
    make_gpx(path, 20)
    with open(path + ".pos.tmp", "w") as f:
        f.write("9999\n")
    gl.up_reset()
    gl.up["url"] = "https://dawarich.example.com"
    gl.up["key"] = "secret-key"
    gl.wifi["state"] = "connected"
    gl.up["next_at"] = _clock[0]
    check("powerloss: up_reset removes leftover .pos.tmp",
          not os.path.exists(path + ".pos.tmp"))
    check("powerloss: missing .pos means offset 0",
          gl.read_offset(path) == 0)
    pump()
    check("powerloss: file is POSTed (not dropped)",
          len(_posts) == 1 and not os.path.exists(path),
          "posts=%s exists=%s state=%s" % (len(_posts), os.path.exists(path), gl.up["state"]))

    # --- corrupt .pos (past EOF) must reset to 0, never delete without POST
    online(tmpdir)
    path = os.path.join(tmpdir, "20261005_132000.gpx")
    make_gpx(path, 12)
    with open(path + ".pos", "w") as f:
        f.write("%d\n" % (os.path.getsize(path) + 500))
    check("corrupt pos: read_offset resets to 0",
          gl.read_offset(path) == 0)
    pump()
    check("corrupt pos: POST happens (file not silently deleted)",
          len(_posts) == 1)
    check("corrupt pos: points uploaded",
          len(locations()) == 12)

    # empty / garbage .pos
    online(tmpdir)
    path = os.path.join(tmpdir, "20261005_133000.gpx")
    make_gpx(path, 4)
    with open(path + ".pos", "w") as f:
        f.write("not-a-number\n")
    check("garbage pos: treated as 0", gl.read_offset(path) == 0)
    pump()
    check("garbage pos: uploaded", len(_posts) == 1)

    # --- HTTP 400: batch quarantine only -----------------------------------
    online(tmpdir)
    path = os.path.join(tmpdir, "20261005_140000.gpx")
    make_gpx(path, 10)
    _post_code[0] = 400
    pump()
    check("http400: batch parked in .gpx.bad after one payload error",
          os.path.exists(path + ".bad"),
          "state=%s" % gl.up["state"])
    check("http400: original GPX gone (whole remaining file was the batch)",
          not os.path.exists(path))
    check("http400: pending() no longer lists the GPX",
          path not in gl.pending())

    # 80-point file: only first batch is parked, rest still uploads
    online(tmpdir)
    path = os.path.join(tmpdir, "20261005_141000.gpx")
    make_gpx(path, 80)
    _post_code[0] = 422
    pump()
    check("http422: GPX remains after batch quarantine",
          os.path.exists(path) and os.path.exists(path + ".bad"))
    with open(path + ".bad", "rb") as f:
        bad_n = sum(1 for line in f if line.startswith(b"<trkpt"))
    check("http422: 50 points in .bad (one batch)", bad_n == 50)
    _post_code[0] = 201
    pump()
    check("http422: remaining 30 points uploaded",
          len(_posts) == 2 and len(json.loads(_posts[1]["data"].decode())["locations"]) == 30)
    check("http422: GPX deleted after the good tail",
          not os.path.exists(path))

    # --- 404 / 403 / 429 / 302 / 5xx: no quarantine, backoff ---------------
    for code, name in ((404, "http404"), (403, "http403"), (429, "http429"),
                       (302, "redirect"), (500, "http500")):
        online(tmpdir)
        path = os.path.join(tmpdir, "20261005_15%s.gpx" % code)
        make_gpx(path, 8)
        _post_code[0] = code
        if code == 429:
            _post_headers[0] = {"Retry-After": "7"}
        pump()
        check("%s: GPX not quarantined" % name,
              os.path.exists(path) and not os.path.exists(path + ".bad"),
              "state=%s" % gl.up["state"])
        if code == 429:
            check("http429: Retry-After 7s clamped to BACKOFF_START",
                  gl.up["next_at"] == _clock[0] + gl.BACKOFF_START,
                  "next_at=%s clock=%s" % (gl.up["next_at"], _clock[0]))
        _post_headers[0] = {}

    # 5xx exponential backoff
    online(tmpdir)
    path = os.path.join(tmpdir, "20261005_15500.gpx")
    make_gpx(path, 5)
    _post_code[0] = 503
    pump()
    b1 = gl.up["backoff"]
    gl.up["next_at"] = _clock[0]
    pump()
    b2 = gl.up["backoff"]
    check("http5xx: exponential backoff grows and is capped",
          b1 == gl.BACKOFF_START * 2 and b2 == b1 * 2 and b2 <= gl.BACKOFF_MAX,
          "b1=%s b2=%s" % (b1, b2))

    # 401 retries with long backoff (does not stop the run)
    online(tmpdir)
    path = os.path.join(tmpdir, "20261005_15401.gpx")
    make_gpx(path, 5)
    _post_code[0] = 401
    pump()
    check("http401: long backoff, not a permanent stop",
          gl.up["state"] == "err 401"
          and gl.up["next_at"] == _clock[0] + gl.AUTH_BACKOFF
          and os.path.exists(path))
    _post_code[0] = 201
    gl.up["next_at"] = _clock[0]
    pump()
    check("http401: later retry can succeed",
          not os.path.exists(path) and gl.up["state"] == "ok")

    # --- POINT_MAX / long device_id ----------------------------------------
    online(tmpdir)
    path = os.path.join(tmpdir, "20261005_16id.gpx")
    make_gpx(path, 3)
    orig_id = gl.DEVICE_ID
    gl.DEVICE_ID = "Dawarich-GPS-tracker"  # 21 chars, the field default
    n, n_ok, size, new_off, eof = gl.read_batch(path, 0)
    check("device_id default encodes all 3 points", n_ok == 3 and size > 0)
    gl.DEVICE_ID = "X" * 32
    n, n_ok, size, new_off, eof = gl.read_batch(path, 0)
    check("32-char device_id: points still encoded (no silent drop)",
          n_ok == 3 and size > 0, "n_ok=%s size=%s" % (n_ok, size))
    body = json.loads(bytes(gl._body[:size]).decode())
    check("32-char device_id appears in JSON",
          body["locations"][0]["properties"]["device_id"] == "X" * 32)
    gl.DEVICE_ID = orig_id

    # inf / nan / bad timestamp skipped
    online(tmpdir)
    path = os.path.join(tmpdir, "20261005_16bad.gpx")
    extras = ["<ele>10.0</ele>", "<ele>inf</ele>", "<ele>1.0</ele>"]
    times = ["2026-10-05T00:00:00Z", "2026-10-05T00:00:01Z",
             '2026-10-05T00:00:02Z"injected']
    make_gpx(path, 3, extras=extras, times=times)
    n, n_ok, size, new_off, eof = gl.read_batch(path, 0)
    check("K6: inf ele omitted but point kept; bad timestamp skipped",
          n == 3 and n_ok == 2, "n=%s n_ok=%s" % (n, n_ok))
    body = json.loads(bytes(gl._body[:size]).decode())
    alts = [loc["properties"].get("altitude") for loc in body["locations"]]
    check("K6: no inf altitude in JSON",
          all(a is None or (a == a and a != float("inf")) for a in alts))

    # --- full-flash path ---------------------------------------------------
    online(tmpdir)
    gl.log["online"] = True
    gl.log["full"] = True
    gl.wifi["state"] = "lost"
    gl.mode_tick()
    check("full: going offline clears log['full']",
          gl.log["full"] is False and gl.log["online"] is False)
    gl.log["file"] = os.path.join(tmpdir, "20261005_160000.gpx")
    gl.log["created"] = False
    gl.log["batch"] = ['<trkpt lat="1" lon="2"><time>t</time></trkpt>\n']
    orig_free = gl.free
    gl.free = lambda: 100
    gl.flush()
    check("full: flush() with MIN_FREE not met sets full and drops batch",
          gl.log["full"] is True and gl.log["batch"] == [])
    gl.free = lambda: gl.MIN_FREE + 10
    gl.log["online"] = False
    gl.log_tick()
    check("full: log_tick clears full when space returns",
          gl.log["full"] is False)
    gl.free = orig_free

    # --- http:// GEO_URL warning -------------------------------------------
    write_env(tmpdir, "SSID=x\nWPA2=y\nGEO_URL=http://dawarich.lan\nAPI_KEY=k\n")
    gl.wifi_reset()
    gl.up_reset()
    reset_wlan()
    WLAN.ok = set()
    buf = []
    real_print = (print if not isinstance(__builtins__, dict)
                  else __builtins__["print"])

    def capturing_print(*a, **k):
        buf.append(" ".join(str(x) for x in a))
        real_print(*a, **k)

    if isinstance(__builtins__, dict):
        __builtins__["print"] = capturing_print
    else:
        __builtins__.print = capturing_print
    try:
        gl.wifi_start()
    finally:
        if isinstance(__builtins__, dict):
            __builtins__["print"] = real_print
        else:
            __builtins__.print = real_print
    check("http GEO_URL: warning printed",
          any("http://" in line and "WARNING" in line for line in buf),
          repr(buf))
    check("http GEO_URL: still accepted for LAN self-host",
          gl.up["url"] == "http://dawarich.lan")
    check("wifi start never logs the password",
          all("y" not in line or "WPA" not in line for line in buf)
          and all("wifi-password" not in line for line in buf))

    # --- count_points from offset + up_reset tmp cleanup -------------------
    online(tmpdir)
    path = os.path.join(tmpdir, "20261005_170000.gpx")
    make_gpx(path, 40)
    # write a sane mid-file offset (start of a line)
    skipped = 0
    with open(path, "rb") as f:
        while skipped < 15:
            line = f.readline()
            if not line:
                break
            if line.startswith(b"<trkpt"):
                skipped += 1
        mid = f.tell()
    gl.write_offset(path, mid)
    left = gl.count_points()
    check("count_points: counts from sidecar offset (not the whole file)",
          0 < left < 40, "left=%s mid=%s" % (left, mid))
    tmpf = os.path.join(tmpdir, "20261005_170000.gpx.pos.tmp")
    with open(tmpf, "w") as f:
        f.write("1\n")
    gl.up_reset()
    check("up_reset: removes leftover .tmp sidecars",
          not os.path.exists(tmpf))

    orphan = os.path.join(tmpdir, "gone.gpx.pos")
    with open(orphan, "w") as f:
        f.write("0\n")
    gl.up_reset()
    check("up_reset: removes orphan .gpx.pos without a matching .gpx",
          not os.path.exists(orphan))

    # --- WDT budget: count_points feeds inside the loop --------------------
    path = os.path.join(tmpdir, "20261005_17wdt.gpx")
    make_gpx(path, 200)
    dog = WDT(timeout=gl.WDT_TIMEOUT)
    gl._wdt = dog
    before = dog.fed
    gl.count_points()
    check("WDT budget: count_points feeds during a long read",
          dog.fed > before, "fed %s -> %s" % (before, dog.fed))
    online(tmpdir)
    make_gpx(path, 200)
    gl._wdt = dog
    before = dog.fed
    pump()
    check("WDT budget: upload_tick feeds around POST",
          dog.fed > before, "fed %s -> %s posts=%s" % (before, dog.fed, len(_posts)))

    # --- crash handler / Ctrl-C / startup ----------------------------------
    gl.log_reset()
    gl.wifi_reset()
    gl.up_reset()
    gl.wifi["wlan"] = None
    _resets[:] = []
    _wdt[:] = []
    _timers[:] = []
    gl._wdt = None

    def boom():
        raise ValueError("bad GEO_URL")

    orig_loop = gl._loop
    gl._loop = boom
    try:
        gl.run()
        check("crash handler: machine.reset() called", False, "run() returned")
    except SystemExit as e:
        check("crash handler: machine.reset() called",
              bool(_resets) and "reset" in str(e))
    except Exception as e:
        check("crash handler: machine.reset() called", False, repr(e))
    finally:
        gl._loop = orig_loop

    # startup exception (before _loop) also resets
    _resets[:] = []
    gl._wdt = None
    orig_reset = gl.reset

    def boom_init():
        raise OSError("oled banner")

    gl.reset = boom_init
    try:
        gl.run()
        check("startup OSError: machine.reset()", False, "returned")
    except SystemExit:
        check("startup OSError: machine.reset()", bool(_resets))
    except Exception as e:
        check("startup OSError: machine.reset()", False, repr(e))
    finally:
        gl.reset = orig_reset

    # Ctrl-C: no reset, Timer keepalive
    _resets[:] = []
    _timers[:] = []
    gl._wdt = None

    def ki():
        raise KeyboardInterrupt

    gl._loop = ki
    try:
        gl.run()
        check("Ctrl-C: raised KeyboardInterrupt", False, "returned")
    except KeyboardInterrupt:
        check("Ctrl-C: raised KeyboardInterrupt (no reset)",
              not _resets)
    except SystemExit:
        check("Ctrl-C: raised KeyboardInterrupt (no reset)", False, "reset")
    finally:
        gl._loop = orig_loop
    check("Ctrl-C: Timer started to keep feeding WDT",
          any(t.period for t in _timers),
          "timers=%s" % [(t.period, t.mode) for t in _timers])

    # _loop creates WDT if run() did not
    _wdt[:] = []
    gl._wdt = None
    gl.wifi["wlan"] = None
    gl.wifi["state"] = "off"
    orig_sleep = gl.time.sleep_ms

    def sleep_and_stop(ms):
        raise KeyboardInterrupt

    gl.time.sleep_ms = sleep_and_stop
    try:
        gl._loop()
    except KeyboardInterrupt:
        pass
    finally:
        gl.time.sleep_ms = orig_sleep
    check("WDT created at start of _loop with 120s timeout",
          gl._wdt is not None and gl._wdt.timeout == 120000 and gl._wdt.fed >= 1,
          "wdt timeout=%s fed=%s" % (
              None if gl._wdt is None else gl._wdt.timeout,
              None if gl._wdt is None else gl._wdt.fed))

    # --- HDOP filter (fresh GGA only) --------------------------------------
    gl.log_reset()
    gl.log["full"] = False
    gl.log["online"] = False
    gl.log["file"] = os.path.join(tmpdir, "now.gpx")
    gl.log["created"] = True
    gl.st.update(valid=True, date="2026-10-05", time="12:00:00",
                 lat=52.5, lon=13.4, alt=10.0, pos_at=_clock[0],
                 hdop=9.0, hdop_at=_clock[0], status="A", quality=1,
                 fixtype=3)
    gl.log["fix"] = True
    gl.log["last_sample"] = None
    gl.log_tick()
    check("HDOP: point with fresh hdop>5 is not recorded",
          gl.log["batch"] == [])
    gl.st["hdop"] = 1.2
    gl.st["hdop_at"] = _clock[0]
    gl.log["last_sample"] = None
    gl.log["last_t"] = None
    gl.log_tick()
    check("HDOP: point with hdop<=5 is recorded",
          len(gl.log["batch"]) == 1)
    check("HDOP: recorded line stores <hdop>",
          "<hdop>" in gl.log["batch"][0])
    # stale hdop from a previous epoch is ignored (point still logged)
    gl.st["hdop"] = 9.0
    gl.st["hdop_at"] = _clock[0] - 10000
    gl.st["time"] = "12:00:01"
    gl.st["lat"] = 52.51
    gl.log["last_sample"] = None
    gl.log_tick()
    check("HDOP: stale hdop does not block a new sample",
          len(gl.log["batch"]) == 2, "batch=%s" % gl.log["batch"])

    # --- OLED dropout at runtime -------------------------------------------
    SSD1306_I2C.raise_show = True
    gl.oled_err = 0
    for _ in range(gl.OLED_MAX_ERR):
        gl.draw()
    check("oled runtime: display disabled after N I2C errors",
          gl.oled is None)
    try:
        gl.draw()
        check("oled runtime: later draw() is a no-op", True)
    except Exception as e:
        check("oled runtime: later draw() is a no-op", False, repr(e))
    SSD1306_I2C.raise_show = False

    # --- multi-WiFi --------------------------------------------------------
    def drain_wifi():
        for _ in range(12):
            if gl.wifi["state"] in ("connected", "not found", "error",
                                    "off", "no .env"):
                return
            _clock[0] += max(gl.CONNECT_TIMEOUT, gl.IGNORE_STALE_MS)
            gl.wifi_tick()

    def start_wifi(env_text, aps, ok):
        reset_wlan()
        WLAN.scan_aps = aps
        WLAN.ok = set(ok)
        write_env(tmpdir, env_text)
        gl.wifi_reset()
        gl.up_reset()
        stale = gl.IGNORE_STALE_MS
        gl.IGNORE_STALE_MS = 0
        gl.wifi_start()
        drain_wifi()
        gl.IGNORE_STALE_MS = stale
        return gl.wifi

    single = "SSID=home\nWPA2=secret1\nGEO_URL=https://d.example\nAPI_KEY=k\n"
    triple = (single + "SSID2=phone\nWPA2_2=secret2\n"
              "SSID3=cabin\nWPA2_3=secret3\n"
              "DEVICE_ID=Dawarich-GPS-tracker\nHOSTNAME=Dawarich-GPS-tracker\n")

    w = start_wifi(single, [("home", -40)], ["home"])
    check("wifi single-network config still connects",
          w["state"] == "connected" and w["ssid"] == "home")
    check("wifi hostname set from default/env",
          "Dawarich-GPS-tracker" in _hostnames)
    check("wifi OLED line shows SSID not password",
          gl._wifi_line() == "WiFi home")

    w = start_wifi(triple, [("phone", -55)], ["phone"])
    check("wifi #1 absent -> #2 used",
          w["state"] == "connected" and w["ssid"] == "phone",
          "state=%s ssid=%s connects=%s" % (w["state"], w["ssid"], WLAN.connects))

    w = start_wifi(triple, [("home", -30), ("phone", -50)], ["phone"])
    check("wifi strongest fails -> second strongest used",
          w["state"] == "connected" and w["ssid"] == "phone",
          "state=%s ssid=%s connects=%s" % (w["state"], w["ssid"], WLAN.connects))
    check("wifi tried strongest first",
          WLAN.connects and WLAN.connects[0][0] == "home")
    # pocket hotspot stronger than home: still try home first at boot
    w = start_wifi(triple, [("phone", -45), ("home", -70)], ["home", "phone"])
    check("home-first: home >= -75 dBm is tried before a stronger hotspot",
          WLAN.connects and WLAN.connects[0][0] == "home"
          and w["ssid"] == "home",
          "connects=%s ssid=%s" % (WLAN.connects, w.get("ssid")))
    w = start_wifi(triple, [("phone", -45), ("home", -80)], ["home", "phone"])
    check("home-first: home below -75 dBm is not preferred",
          WLAN.connects and WLAN.connects[0][0] == "phone",
          "connects=%s" % (WLAN.connects,))
    # duplicate BSSIDs must not produce two tries of the same SSID first
    reset_wlan()
    WLAN.scan_aps = [("home", -20, b"\x01" * 6), ("home", -40, b"\x02" * 6),
                     ("phone", -50, b"\x03" * 6)]
    WLAN.ok = set(["home"])
    write_env(tmpdir, triple)
    gl.wifi_reset()
    gl.up_reset()
    gl.IGNORE_STALE_MS = 0
    gl.wifi_start()
    drain_wifi()
    home_tries = [c for c in WLAN.connects if c[0] == "home"]
    check("wifi duplicate BSSID: home tried once (strongest)",
          len(home_tries) == 1, "connects=%s" % (WLAN.connects,))

    w = start_wifi(triple, [], [])
    check("wifi all absent -> offline (not connected)",
          w["state"] != "connected")
    check("wifi all absent: no blind 1-2-3 tries (none hidden)",
          WLAN.connects == [], "connects=%s" % (WLAN.connects,))
    # logging continues
    gl.log_reset()
    gl.log["online"] = False
    gl.log["full"] = False
    gl.st.update(valid=True, date="2026-10-05", time="18:00:00",
                 lat=52.5, lon=13.4, alt=10.0, pos_at=_clock[0],
                 hdop=1.0, hdop_at=_clock[0], status="A", quality=1,
                 fixtype=3)
    gl.log["fix"] = False
    gl.log["lost_at"] = None
    gl.log["file"] = None
    gl.log_tick()
    check("wifi all absent: offline logging still records a point",
          len(gl.log["batch"]) == 1 or gl.log["file"] is not None,
          "batch=%s file=%s" % (gl.log.get("batch"), gl.log.get("file")))

    # failover after disconnect
    w = start_wifi(triple, [("home", -40), ("phone", -70)], ["home", "phone"])
    check("wifi failover setup connected to home",
          w["state"] == "connected" and w["ssid"] == "home")
    WLAN.ok = set(["phone"])
    if WLAN.instances:
        WLAN.instances[-1]._status = 5  # beacon timeout
    WLAN.connects = []
    gl.wifi_tick()
    check("wifi loss first leaves LOG (not blocking connect)",
          gl.wifi["state"] != "connected" and gl.at_home() is False)
    gl.mode_tick()
    gl.IGNORE_STALE_MS = 0
    drain_wifi()
    check("wifi failover after disconnect uses the other known net",
          gl.wifi["state"] == "connected" and gl.wifi["ssid"] == "phone",
          "state=%s ssid=%s connects=%s" % (
              gl.wifi["state"], gl.wifi["ssid"], WLAN.connects))

    # scan error: config order 1 then 2
    reset_wlan()
    WLAN.scan_error = True
    WLAN.ok = set(["phone"])
    write_env(tmpdir, triple)
    gl.wifi_reset()
    gl.up_reset()
    gl.IGNORE_STALE_MS = 0
    gl.wifi_start()
    drain_wifi()
    check("wifi scan fail: config order reaches #2",
          gl.wifi["state"] == "connected" and gl.wifi["ssid"] == "phone",
          "state=%s ssid=%s" % (gl.wifi["state"], gl.wifi["ssid"]))

    # .bad recovery after a successful upload
    online(tmpdir)
    path = os.path.join(tmpdir, "20261005_18bad.gpx")
    make_gpx(path, 4)
    _post_code[0] = 400
    pump()
    check("recovery: payload error created .bad",
          os.path.exists(path + ".bad"))
    _post_code[0] = 201
    gl.up["allow_bad"] = True
    gl.up["bad_try"] = {}
    pump()
    check("recovery: .bad retried and consumed after success path",
          not os.path.exists(path + ".bad") or gl.up["sent"] > 0,
          "bad=%s sent=%s posts=%s" % (
              os.path.exists(path + ".bad"), gl.up["sent"], len(_posts)))

    # --- B1: persistent 400 on 2000-pt file must not drop points -----------
    online(tmpdir)
    path = os.path.join(tmpdir, "20261005_19e1.gpx")
    make_gpx(path, 2000)
    _post_code[0] = 400
    for _ in range(8):
        pump()
        gl.up["next_at"] = _clock[0]
    remain = 0
    if os.path.exists(path):
        remain += gl.count_points()
    badp = path + ".bad"
    bad_n = 0
    if os.path.exists(badp):
        with open(badp, "rb") as f:
            bad_n = sum(1 for line in f if line.startswith(b"<trkpt"))
    check("B1 E1: persistent 400 does not delete unsent points",
          os.path.exists(path) and remain + bad_n == 2000,
          "remain=%s bad=%s exists=%s state=%s" % (
              remain, bad_n, os.path.exists(path), gl.up["state"]))
    check("B1 E1: global payload errors stop quarantining",
          gl.up.get("fail4", 0) >= gl.MAX_GLOBAL_PAYLOAD
          and bad_n <= gl.UPLOAD_BATCH * (gl.MAX_GLOBAL_PAYLOAD - 1),
          "fail4=%s bad_n=%s" % (gl.up.get("fail4"), bad_n))

    # --- B1 E2: .bad retry + 422 must keep the quarantined file ------------
    online(tmpdir)
    path = os.path.join(tmpdir, "20261005_19e2.gpx")
    make_gpx(path, 10)
    _post_code[0] = 422
    pump()
    check("B1 E2: first 422 parked the batch", os.path.exists(path + ".bad"))
    _post_code[0] = 422
    gl.up["allow_bad"] = True
    gl.up["next_at"] = _clock[0]
    pump()
    check("B1 E2: .bad still on flash after retry 422",
          os.path.exists(path + ".bad"),
          "state=%s" % gl.up["state"])
    with open(path + ".bad", "rb") as f:
        still = sum(1 for line in f if line.startswith(b"<trkpt"))
    check("B1 E2: quarantined points not deleted",
          still == 10, "still=%s" % still)

    # --- B1: .bad full: do not advance offset ------------------------------
    online(tmpdir)
    path = os.path.join(tmpdir, "20261005_19full.gpx")
    make_gpx(path, 80)
    with open(path + ".bad", "wb") as f:
        f.write(b"x" * (gl.MAX_BAD_BYTES))
    _post_code[0] = 400
    off0 = gl.read_offset(path)
    pump()
    check("B1 bad-full: offset not advanced",
          gl.read_offset(path) == off0 and os.path.exists(path),
          "off=%s state=%s" % (gl.read_offset(path), gl.up["state"]))
    check("B1 bad-full: state is bad full",
          gl.up["state"] == "bad full")
    check("B1 bad-full: OLED mode BAD FULL",
          gl._mode_line() == "Mode BAD FULL")
    later = os.path.join(tmpdir, "20261005_19later.gpx")
    make_gpx(later, 8)
    _posts[:] = []
    _post_code[0] = 201
    gl.up["next_at"] = _clock[0]
    pump()
    check("B1: another file's 201 does not clear BAD FULL",
          gl.up["state"] == "bad full",
          "state=%s sent=%s" % (gl.up["state"], gl.up["sent"]))
    check("B1: OLED stays BAD FULL after another file succeeds",
          gl._mode_line() == "Mode BAD FULL")
    with open(path, "rb") as f:
        kept = sum(1 for line in f if line.startswith(b"<trkpt"))
    check("B1 bad-full: original 80 points still in GPX",
          kept == 80)

    # --- B2: 200 kB line without newline must not MemoryError/boot-loop ----
    online(tmpdir)
    path = os.path.join(tmpdir, "20261005_20long.gpx")
    with open(path, "wb") as f:
        f.write((gl.HEADER % "20261005_20long").encode())
        f.write(b'<trkpt lat="52.500000" lon="13.400000">'
                b'<time>2026-10-05T00:00:00Z</time></trkpt>\n')
        f.write(b"A" * 200000 + b"\n")
        f.write(b'<trkpt lat="52.500100" lon="13.400100">'
                b'<time>2026-10-05T00:00:01Z</time></trkpt>\n')
        f.write(gl.FOOTER.encode())
    n, n_ok, size, new_off, eof = gl.read_batch(path, 0)
    check("B2: 200 kB over-long line skipped, both points encoded",
          n_ok == 2 and eof,
          "n=%s n_ok=%s eof=%s" % (n, n_ok, eof))
    left = gl.count_points()
    check("B2: count_points survives a 200 kB line",
          left == 2, "left=%s" % left)
    memerr = []
    orig_rl = gl._readline

    def boom_line(f):
        memerr.append(1)
        raise MemoryError("heap")

    gl._readline = boom_line
    try:
        gl.up_reset()
        check("B2: count_points/up_reset catch MemoryError (no reset)",
              True)
    except MemoryError:
        check("B2: count_points/up_reset catch MemoryError (no reset)", False)
    finally:
        gl._readline = orig_rl
        gl.up["url"] = "https://dawarich.example.com"
        gl.up["key"] = "secret-key"
        gl.wifi["state"] = "connected"

    gl._readline = boom_line
    try:
        pump()
        check("B2: upload_tick MemoryError skips the file (no infinite loop)",
              gl.up["skip"] == path and gl.up["state"] == "low mem")
    except MemoryError:
        check("B2: upload_tick MemoryError skips the file (no infinite loop)",
              False, "MemoryError escaped")
    finally:
        gl._readline = orig_rl

    # --- B3: hotspot (non-home) keeps logging + opportunistic upload ------
    def gps_fix(lat=52.5):
        gl.st.update(valid=True, date="2026-10-05", time="12:00:00",
                     lat=lat, lon=13.4, alt=10.0, pos_at=_clock[0],
                     hdop=1.0, hdop_at=_clock[0], status="A", quality=1,
                     fixtype=3)

    env_hot = ("SSID=home\nWPA2=h\nSSID2=PhoneHotspot\nWPA2_2=p\n"
               "GEO_URL=https://d.example\nAPI_KEY=k\n")
    w = start_wifi(env_hot, [("PhoneHotspot", -40)], ["PhoneHotspot"])
    check("B3: connected to non-home hotspot",
          w["state"] == "connected" and w["ssid"] == "PhoneHotspot"
          and w["home"] is False)
    check("B3: at_home() is False on hotspot", gl.at_home() is False)
    gl.log_reset()
    gl.log["online"] = True
    gl.mode_tick()
    check("B3: mode_tick does not enter upload-only on hotspot",
          gl.log["online"] is False)
    gps_fix(52.50)
    gl.log["fix"] = False
    gl.log["lost_at"] = None
    gl.log["file"] = None
    gl.log["last_sample"] = None
    gl.log_tick()
    check("B3: hotspot trip still records GPS points",
          len(gl.log["batch"]) == 1 or gl.log["file"] is not None,
          "batch=%s file=%s" % (gl.log.get("batch"), gl.log.get("file")))
    check("B3: OLED mode LOG+UP on hotspot",
          gl._mode_line() == "Mode LOG+UP")

    current = gl.log.get("file")
    for n in os.listdir(tmpdir):
        p = os.path.join(tmpdir, n)
        if p == current:
            continue
        if n.endswith((".gpx", ".pos", ".tmp", ".bad")):
            try:
                os.remove(p)
            except OSError:
                pass
    finished = os.path.join(tmpdir, "20261004_210000.gpx")
    make_gpx(finished, 4)
    gl.up["url"] = "https://d.example"
    gl.up["key"] = "k"
    gl.up["next_at"] = _clock[0]
    gl.up["state"] = "idle"
    _post_code[0] = 201
    _posts[:] = []
    pump()
    check("B3: hotspot still uploads finished files in background",
          len(_posts) == 1 and not os.path.exists(finished),
          "posts=%s exists=%s state=%s" % (
              len(_posts), os.path.exists(finished), gl.up["state"]))

    w = start_wifi(env_hot, [("home", -30)], ["home"])
    check("B3: home network is marked home",
          w["state"] == "connected" and w["home"] is True)
    gl.log_reset()
    gl.log["online"] = False
    gl.mode_tick()
    check("B3: home SSID stops logging (upload-only)",
          gl.log["online"] is True)
    gps_fix(52.51)
    gl.log["last_sample"] = None
    gl.log_tick()
    check("B3: log_tick is a no-op on home",
          gl.log["batch"] == [] and gl.log["file"] is None)
    check("B3: OLED mode UPLOAD on home",
          gl._mode_line() == "Mode UPLOAD")

    # HOME2 override
    env_h2 = env_hot + "HOME2=1\n"
    w = start_wifi(env_h2, [("PhoneHotspot", -40)], ["PhoneHotspot"])
    check("B3: HOME2=1 treats hotspot as home",
          w["home"] is True, "home=%s" % w["home"])

    # --- K1: keepalive deinit + deadline -----------------------------------
    _resets[:] = []
    _timers[:] = []
    gl._wdt = WDT(timeout=gl.WDT_TIMEOUT)
    gl._keep = None
    gl._running = False
    orig_loop = gl._loop

    def ki():
        raise KeyboardInterrupt

    gl._loop = ki
    try:
        gl.run()
    except KeyboardInterrupt:
        pass
    finally:
        gl._loop = orig_loop
    t0 = _timers[-1] if _timers else None
    check("K1: Ctrl-C starts a keepalive Timer",
          t0 is not None and t0.callback is not None)
    fed0 = gl._wdt.fed
    t0.callback(t0)
    check("K1: callback feeds WDT before the deadline",
          gl._wdt.fed > fed0)
    _clock[0] += gl.KEEPALIVE_MAX + 1
    t0.callback(t0)
    check("K1: after KEEPALIVE_MAX the timer is deinited",
          getattr(t0, "deinited", False) is True)
    fed1 = gl._wdt.fed
    t0.callback(t0)
    check("K1: after deadline the WDT is no longer fed",
          gl._wdt.fed == fed1)

    gl._wdt = WDT(timeout=gl.WDT_TIMEOUT)
    gl._keep = t0
    t0.deinited = False
    gl._running = False
    orig_loop2 = gl._loop

    def stop_now():
        raise KeyboardInterrupt

    gl._loop = stop_now
    try:
        gl.run()
    except KeyboardInterrupt:
        pass
    finally:
        gl._loop = orig_loop2
    check("K1: run() deinit's the previous keepalive Timer",
          t0.deinited is True)

    gl._running = True
    gl._keep_from = _clock[0]
    fed2 = gl._wdt.fed
    if _timers:
        cb = _timers[-1].callback
        if cb:
            cb(_timers[-1])
    check("K1: callback does not feed while _running",
          gl._wdt.fed == fed2)
    gl._running = False

    # --- K2: boot-failure counter ------------------------------------------
    RTC.mem = b""
    boot_path = os.path.join(ROOT, "main.py")
    boot_src = open(boot_path).read()
    boot_ns = {
        "Pin": Pin, "RTC": RTC, "machine": sys.modules["machine"],
        "time": time, "__name__": "boot_helpers",
    }
    exec(compile(boot_src, boot_path, "exec"), boot_ns)
    check("K2: fresh boot-fail counter is 0",
          boot_ns["_boot_fail_get"]() == 0)
    boot_ns["_boot_fail_set"](2)
    check("K2: RTC memory stores the counter",
          boot_ns["_boot_fail_get"]() == 2)
    check("K2: delay grows with the counter and caps at 30s",
          boot_ns["_boot_fail_delay"](1) == 5
          and boot_ns["_boot_fail_delay"](2) == 10
          and boot_ns["_boot_fail_delay"](10) == 30)
    _resets[:] = []
    RTC.mem = b"4"
    did = boot_ns["_boot_handle_error"]()
    check("K2: 5th failure drops to REPL (no reset)",
          did is False and boot_ns["_boot_fail_get"]() >= 5)
    RTC.mem = b"%d" % boot_ns["BOOT_FAIL_MAX"]
    printed = []
    real_print = (print if not isinstance(__builtins__, dict)
                  else __builtins__["print"])

    def cap(*a, **k):
        printed.append(" ".join(str(x) for x in a))

    if isinstance(__builtins__, dict):
        __builtins__["print"] = cap
    else:
        __builtins__.print = cap
    try:
        boot_ns["_boot_start"]()
    finally:
        if isinstance(__builtins__, dict):
            __builtins__["print"] = real_print
        else:
            __builtins__.print = real_print
    check("K2: at BOOT_FAIL_MAX the logger is not started (REPL)",
          any("REPL" in line for line in printed),
          repr(printed))

    # --- K3: delayed CONNECTING does not block GPS sampling ----------------
    reset_wlan()
    WLAN.scan_aps = [("PhoneHotspot", -40)]
    WLAN.ok = set(["PhoneHotspot"])
    WLAN.delay = 8
    write_env(tmpdir, env_hot)
    gl.wifi_reset()
    gl.up_reset()
    gl.IGNORE_STALE_MS = 0
    t_before = _clock[0]
    gl.wifi_start()
    dt = _clock[0] - t_before
    check("K3: wifi_start does not sleep CONNECT_TIMEOUT",
          dt < gl.CONNECT_TIMEOUT,
          "dt=%s state=%s" % (dt, gl.wifi["state"]))
    check("K3: still connecting after one tick",
          gl.wifi["state"] == "connecting")
    gl.log_reset()
    gl.log["online"] = False
    gps_fix(52.52)
    gl.log["fix"] = False
    gl.log["lost_at"] = None
    gl.log["file"] = None
    gl.log["last_sample"] = None
    gl.log_tick()
    check("K3: GPS sampling continues while WiFi is CONNECTING",
          len(gl.log["batch"]) == 1 or gl.log["file"] is not None,
          "batch=%s file=%s" % (gl.log.get("batch"), gl.log.get("file")))
    # one wifi_tick must not jump CONNECT_TIMEOUT of clock
    t_before = _clock[0]
    gl.wifi_tick()
    check("K3: wifi_tick is non-blocking (no 12s sleep)",
          _clock[0] - t_before < 1000,
          "dt=%s" % (_clock[0] - t_before))
    WLAN.delay = 0

    # --- K4: point that does not fit must not drop the remainder -----------
    online(tmpdir)
    path = os.path.join(tmpdir, "20261005_22k4.gpx")
    make_gpx(path, 100)
    orig_id = gl.DEVICE_ID
    orig_body = gl._body
    gl.DEVICE_ID = "X" * 32
    gl._body = bytearray(16 * 256 + 32)
    n, n_ok, size, new_off, eof = gl.read_batch(path, 0)
    check("K4: undersized buffer returns eof=False when a point does not fit",
          n_ok > 0 and n_ok < 50 and eof is False,
          "n=%s n_ok=%s eof=%s" % (n, n_ok, eof))
    _post_code[0] = 201
    gl.up["next_at"] = _clock[0]
    # simulate commit of that partial batch
    gl.commit_offset(path, n, new_off, eof)
    check("K4: commit_offset does not delete the remainder",
          os.path.exists(path),
          "exists=%s eof=%s" % (os.path.exists(path), eof))
    rest = 0
    with open(path, "rb") as f:
        f.seek(new_off)
        while True:
            line = f.readline()
            if not line:
                break
            if line.startswith(b"<trkpt"):
                rest += 1
    check("K4: unread points remain in the file",
          n_ok + rest == 100,
          "n_ok=%s rest=%s" % (n_ok, rest))
    gl.DEVICE_ID = orig_id
    gl._body = orig_body

    # --- K5: DEVICE_ID / HOSTNAME validation -------------------------------
    check("K5: quote in DEVICE_ID falls back to default",
          gl._safe_name('foo"bar', "Dawarich-GPS-tracker")
          == "Dawarich-GPS-tracker")
    check("K5: backslash in DEVICE_ID falls back",
          gl._safe_name("foo\\bar", "Dawarich-GPS-tracker")
          == "Dawarich-GPS-tracker")
    check("K5: 80-char DEVICE_ID falls back",
          gl._safe_name("X" * 80, "Dawarich-GPS-tracker")
          == "Dawarich-GPS-tracker")
    check("K5: valid 32-char id accepted",
          gl._safe_name("A" * 32, "d") == "A" * 32)
    check("K5: hostname rejects underscore and dots",
          gl._safe_name("bad_name.host", "Dawarich-GPS-tracker", host=True)
          == "Dawarich-GPS-tracker")
    check("K5: hostname accepts hyphen",
          gl._safe_name("esp32-gps", "d", host=True) == "esp32-gps")
    write_env(tmpdir, 'SSID=home\nWPA2=x\nDEVICE_ID="quoted"\n'
              "GEO_URL=https://d.example\nAPI_KEY=k\n")
    gl.wifi_reset()
    gl.up_reset()
    reset_wlan()
    WLAN.ok = set()
    gl.wifi_start()
    # quotes are stripped by load_env, "quoted" becomes quoted (valid)
    check("K5: surrounding quotes stripped by load_env",
          gl.DEVICE_ID == "quoted")
    write_env(tmpdir, "SSID=home\nWPA2=x\nDEVICE_ID=bad\"id\n"
              "GEO_URL=https://d.example\nAPI_KEY=k\n")
    gl.wifi_reset()
    gl.wifi_start()
    check("K5: embedded quote in DEVICE_ID uses default (valid JSON)",
          gl.DEVICE_ID == "Dawarich-GPS-tracker")
    loc = ('{"type":"Feature","geometry":{"type":"Point",'
           '"coordinates":[13.4,52.5]},"properties":{'
           '"timestamp":"2026-10-05T00:00:00Z","device_id":"%s"}}'
           % gl.DEVICE_ID)
    json.loads('{"locations":[%s]}' % loc)
    check("K5: default device_id produces valid JSON", True)

    # --- K6: hidden SSID tried after / when nothing visible ----------------
    env_hid = (env_hot + "SSID2_HIDDEN=1\n")
    w = start_wifi(env_hid, [("home", -40)], ["home"])
    # visible home first; hidden phone after if home fails
    reset_wlan()
    WLAN.scan_aps = [("home", -40)]
    WLAN.ok = set(["PhoneHotspot"])
    write_env(tmpdir, env_hid)
    gl.wifi_reset()
    gl.up_reset()
    gl.IGNORE_STALE_MS = 0
    gl.wifi_start()
    drain_wifi()
    check("K6 W3: hidden #2 tried after visible #1 fails",
          gl.wifi["state"] == "connected" and gl.wifi["ssid"] == "PhoneHotspot",
          "state=%s ssid=%s connects=%s" % (
              gl.wifi["state"], gl.wifi["ssid"], WLAN.connects))
    reset_wlan()
    WLAN.scan_aps = []  # none of ours visible
    WLAN.ok = set(["PhoneHotspot"])
    write_env(tmpdir, env_hid)
    gl.wifi_reset()
    gl.up_reset()
    gl.IGNORE_STALE_MS = 0
    gl.wifi_start()
    drain_wifi()
    check("K6: empty scan tries only the hidden net",
          WLAN.connects and all(c[0] == "PhoneHotspot" for c in WLAN.connects),
          "connects=%s" % (WLAN.connects,))
    check("K6: hidden net connects when not in the scan",
          gl.wifi["state"] == "connected" and gl.wifi["ssid"] == "PhoneHotspot")

    # --- K7: leftover fail reason is ignored for IGNORE_STALE_MS -----------
    reset_wlan()
    WLAN.scan_aps = [("home", -30), ("PhoneHotspot", -50)]
    WLAN.ok = set(["PhoneHotspot"])
    WLAN.stale_fail = 2
    WLAN.leftover = 4  # STAT_WRONG_PASSWORD leftover
    write_env(tmpdir, env_hot)
    gl.wifi_reset()
    gl.up_reset()
    # keep IGNORE_STALE_MS so leftover is ignored
    gl.wifi_start()
    # first poll may still be leftover; advance a little but not the timeout
    for _ in range(6):
        _clock[0] += 200
        gl.wifi_tick()
        if gl.wifi["state"] == "connected":
            break
    check("K7: leftover status does not immediately fail a good net",
          gl.wifi["state"] in ("connecting", "connected"),
          "state=%s ssid=%s connects=%s" % (
              gl.wifi["state"], gl.wifi.get("ssid"), WLAN.connects))
    WLAN.stale_fail = 0
    drain_wifi()
    check("K7: good net still reached after leftover clears",
          gl.wifi["state"] == "connected",
          "state=%s ssid=%s" % (gl.wifi["state"], gl.wifi.get("ssid")))

    # --- E4 / E5 Retry-After clamp -----------------------------------------
    online(tmpdir)
    path = os.path.join(tmpdir, "20261005_23ra.gpx")
    make_gpx(path, 5)
    _post_code[0] = 429
    _post_headers[0] = {"Retry-After": "-3"}
    pump()
    check("E4: negative Retry-After clamped to BACKOFF_START",
          gl.up["next_at"] == _clock[0] + gl.BACKOFF_START,
          "next_at=%s" % gl.up["next_at"])
    online(tmpdir)
    path = os.path.join(tmpdir, "20261005_24ra.gpx")
    make_gpx(path, 5)
    _post_code[0] = 503
    _post_headers[0] = {"Retry-After": "604800"}
    pump()
    check("E5: 7-day Retry-After clamped to BACKOFF_MAX",
          gl.up["next_at"] == _clock[0] + gl.BACKOFF_MAX,
          "next_at=%s" % gl.up["next_at"])
    _post_headers[0] = {}

    # --- orphan .gpx.bad.pos -----------------------------------------------
    online(tmpdir)
    orphan_bad = os.path.join(tmpdir, "gone.gpx.bad.pos")
    with open(orphan_bad, "w") as f:
        f.write("0\n")
    gl.up_reset()
    gl.up["url"] = "https://dawarich.example.com"
    gl.up["key"] = "secret-key"
    check("orphan .gpx.bad.pos cleaned by up_reset",
          not os.path.exists(orphan_bad))

    # HOME1=0 override
    env_nh = "SSID=cafe\nWPA2=x\nHOME1=0\nGEO_URL=https://d.example\nAPI_KEY=k\n"
    w = start_wifi(env_nh, [("cafe", -40)], ["cafe"])
    check("HOME1=0: network #1 is not home",
          w["home"] is False, "home=%s" % w["home"])

    # --- REG1: no str.isalnum (MicroPython 1.29) ---------------------------
    src = open(os.path.join(ROOT, "gps_logger.py")).read()
    check("REG1: gps_logger.py does not call str.isalnum()",
          ".isalnum(" not in src)
    check("REG1: _ok_char accepts the README default id",
          gl._safe_name("Dawarich-GPS-tracker", "x")
          == "Dawarich-GPS-tracker")
    check("REG1: _ok_char rejects a quote without isalnum",
          gl._safe_name('bad"id', "Dawarich-GPS-tracker")
          == "Dawarich-GPS-tracker")

    # --- roam home from hotspot --------------------------------------------
    env_hot = ("SSID=home\nWPA2=h\nSSID2=PhoneHotspot\nWPA2_2=p\n"
               "GEO_URL=https://d.example\nAPI_KEY=k\n")
    w = start_wifi(env_hot, [("PhoneHotspot", -40)], ["PhoneHotspot"])
    check("roam setup: on hotspot",
          w["ssid"] == "PhoneHotspot" and w["home"] is False)
    WLAN.scan_aps = [("home", -40), ("PhoneHotspot", -50)]
    WLAN.ok = set(["home", "PhoneHotspot"])
    WLAN.connects = []
    gl.wifi["roam_at"] = _clock[0] - gl.ROAM_CHECK
    gl.wifi_tick()
    drain_wifi()
    check("roam: switches to visible home >= -75 dBm",
          gl.wifi["state"] == "connected" and gl.wifi["ssid"] == "home"
          and gl.at_home() is True,
          "state=%s ssid=%s" % (gl.wifi["state"], gl.wifi.get("ssid")))

    w = start_wifi(env_hot, [("PhoneHotspot", -40)], ["PhoneHotspot"])
    WLAN.scan_aps = [("home", -40), ("PhoneHotspot", -50)]
    WLAN.ok = set(["PhoneHotspot"])  # home connect fails
    WLAN.connects = []
    gl.wifi["roam_at"] = _clock[0] - gl.ROAM_CHECK
    gl.wifi_tick()
    drain_wifi()
    check("roam: failed home switch returns to the hotspot",
          gl.wifi["state"] == "connected" and gl.wifi["ssid"] == "PhoneHotspot",
          "state=%s ssid=%s connects=%s" % (
              gl.wifi["state"], gl.wifi.get("ssid"), WLAN.connects))

    w = start_wifi(env_hot, [("PhoneHotspot", -40)], ["PhoneHotspot"])
    WLAN.scan_aps = [("home", -80), ("PhoneHotspot", -40)]
    WLAN.ok = set(["home", "PhoneHotspot"])
    WLAN.connects = []
    gl.wifi["roam_at"] = _clock[0] - gl.ROAM_CHECK
    gl.wifi_tick()
    check("roam: weak home (< -75 dBm) is ignored",
          gl.wifi["ssid"] == "PhoneHotspot" and WLAN.connects == [],
          "ssid=%s connects=%s" % (gl.wifi.get("ssid"), WLAN.connects))

    # exponential roam backoff (wrong home PSK / unseen home)
    w = start_wifi(env_hot, [("PhoneHotspot", -40)], ["PhoneHotspot"])
    WLAN.scan_aps = [("home", -40), ("PhoneHotspot", -50)]
    WLAN.ok = set(["PhoneHotspot"])
    WLAN.connects = []
    gl.st["speed"] = None
    gl.wifi["roam_at"] = _clock[0] - gl.ROAM_CHECK
    gl.wifi_tick()
    drain_wifi()
    check("roam backoff: failed home switch doubles the interval",
          (gl.wifi.get("roam_backoff") or 0) >= gl.ROAM_CHECK * 2,
          "roam_backoff=%s" % gl.wifi.get("roam_backoff"))
    WLAN.connects = []
    if WLAN.instances:
        WLAN.instances[-1]._status = 3
    gl.wifi["state"] = "connected"
    gl.wifi["home"] = False
    gl.wifi["ssid"] = "PhoneHotspot"
    gl.wifi["roam_at"] = _clock[0] - gl.ROAM_CHECK
    gl.wifi_tick()
    check("roam backoff: 5 min later does not retry a failed roam",
          WLAN.connects == [],
          "connects=%s backoff=%s" % (WLAN.connects, gl.wifi.get("roam_backoff")))
    WLAN.connects = []
    gl.wifi["roam_at"] = _clock[0] - gl.wifi["roam_backoff"]
    gl.wifi_tick()
    check("roam backoff: retries after the doubled interval",
          WLAN.connects and WLAN.connects[0][0] == "home",
          "connects=%s" % (WLAN.connects,))
    WLAN.scan_aps = [("PhoneHotspot", -40)]
    WLAN.connects = []
    b_unseen = gl.wifi.get("roam_backoff")
    gl.wifi["roam_at"] = _clock[0] - (b_unseen or gl.ROAM_CHECK)
    gl.wifi_tick()
    check("roam backoff: unseen home also backs off (fewer scans)",
          (gl.wifi.get("roam_backoff") or 0) > b_unseen
          and WLAN.connects == [],
          "before=%s after=%s" % (b_unseen, gl.wifi.get("roam_backoff")))
    gl.st["speed"] = 50
    WLAN.scan_aps = [("home", -40), ("PhoneHotspot", -50)]
    WLAN.ok = set(["home", "PhoneHotspot"])
    WLAN.connects = []
    gl.wifi["roam_backoff"] = gl.ROAM_CHECK
    gl.wifi["roam_at"] = _clock[0] - gl.ROAM_CHECK
    gl.wifi_tick()
    check("roam: skip scan while moving >= 5 km/h",
          WLAN.connects == [],
          "connects=%s" % (WLAN.connects,))
    gl.st["speed"] = None

    # --- K-LL: v1.29 beacon timeout is CONNECTING --------------------------
    w = start_wifi(env_hot, [("home", -30), ("PhoneHotspot", -70)],
                   ["home", "PhoneHotspot"])
    check("LL setup: on home", w["ssid"] == "home")
    if WLAN.instances:
        WLAN.instances[-1]._status = 1  # STAT_CONNECTING, as real v1.29
        WLAN.instances[-1]._recover = 3
    WLAN.connects = []
    gl.wifi_tick()
    check("LL1: first drop is 'lost', not a stale connecting fail",
          gl.wifi["state"] == "lost" and "home" not in gl.wifi["failed"])
    gl.mode_tick()
    for _ in range(3):
        gl.wifi_tick()
    check("LL1: IDF auto-reconnect restores home within grace",
          gl.wifi["state"] == "connected" and gl.wifi["ssid"] == "home"
          and gl.at_home() is True,
          "state=%s ssid=%s failed=%s" % (
              gl.wifi["state"], gl.wifi.get("ssid"), gl.wifi["failed"]))

    w = start_wifi(env_hot, [("home", -30), ("PhoneHotspot", -70)],
                   ["home", "PhoneHotspot"])
    if WLAN.instances:
        WLAN.instances[-1]._status = 1
        WLAN.instances[-1]._recover = 0  # stays CONNECTING
    WLAN.ok = set(["PhoneHotspot"])
    WLAN.connects = []
    gl.wifi_tick()
    _clock[0] += gl.LOST_GRACE
    gl.IGNORE_STALE_MS = 0
    drain_wifi()
    check("LL1 long drop: after grace, failover does not use stale try_at",
          gl.wifi["state"] == "connected" and gl.wifi["ssid"] == "PhoneHotspot",
          "state=%s ssid=%s" % (gl.wifi["state"], gl.wifi.get("ssid")))

    # --- K7 leftover: do not fail a good net / double backoff --------------
    reset_wlan()
    WLAN.scan_aps = [("home", -40)]
    WLAN.ok = set(["home"])
    WLAN.stale_fail = 4
    WLAN.leftover = 4
    write_env(tmpdir, "SSID=home\nWPA2=h\nGEO_URL=https://d.example\nAPI_KEY=k\n")
    gl.wifi_reset()
    gl.up_reset()
    gl.wifi_start()
    b0 = gl.wifi["backoff"]
    for _ in range(6):
        _clock[0] += 200
        gl.wifi_tick()
        if gl.wifi["state"] == "connected":
            break
    check("K7: leftover AUTH_FAIL does not mark a good net failed",
          gl.wifi["state"] == "connected" and "home" not in gl.wifi["failed"],
          "state=%s failed=%s" % (gl.wifi["state"], gl.wifi["failed"]))
    check("K7: leftover does not double the reconnect backoff",
          gl.wifi["backoff"] == gl.WIFI_RETRY,
          "backoff=%s start=%s" % (gl.wifi["backoff"], b0))

    # --- connect() OSError moves to the next candidate ---------------------
    reset_wlan()
    WLAN.scan_aps = [("home", -20), ("PhoneHotspot", -40)]
    WLAN.ok = set(["PhoneHotspot"])
    WLAN.connect_error = "home"
    write_env(tmpdir, env_hot)
    gl.wifi_reset()
    gl.up_reset()
    gl.IGNORE_STALE_MS = 0
    gl.wifi_start()
    drain_wifi()
    check("K3-1: connect() OSError fails over to the next SSID",
          gl.wifi["state"] == "connected" and gl.wifi["ssid"] == "PhoneHotspot",
          "state=%s ssid=%s" % (gl.wifi["state"], gl.wifi.get("ssid")))
    WLAN.connect_error = None

    # --- per-file backoff: one stuck file does not block the others --------
    online(tmpdir)
    stuck = os.path.join(tmpdir, "20261001_000000.gpx")
    later = os.path.join(tmpdir, "20261005_000000.gpx")
    make_gpx(stuck, 200)
    _post_code[0] = 400
    pumps = 0
    while stuck not in (gl.up.get("hold") or {}) and pumps < 8:
        pump()
        gl.up["next_at"] = _clock[0]
        pumps += 1
    check("HOL: oldest file is held after global payload errors",
          stuck in (gl.up.get("hold") or {}),
          "hold=%s pumps=%s" % (gl.up.get("hold"), pumps))
    make_gpx(later, 8)
    _posts[:] = []
    _post_code[0] = 201
    gl.up["next_at"] = _clock[0]
    gl.up["sent"] = 0
    pump()
    check("HOL: newer file uploads while the oldest is held",
          (not os.path.exists(later) or gl.up["sent"] > 0)
          and os.path.exists(stuck),
          "later=%s sent=%s stuck=%s posts=%s" % (
              os.path.exists(later), gl.up["sent"], os.path.exists(stuck),
              len(_posts)))

    # --- hotspot-only rotate; keep fix; never drop a failed write ----------
    rot_dir = os.path.join(tmpdir, "rot")
    os.mkdir(rot_dir)
    gl.LOG_DIR = rot_dir
    open_path = os.path.join(rot_dir, "open.gpx")
    make_gpx(open_path, 3)
    gl.log_reset()
    gl.log["online"] = False
    gl.wifi["state"] = "connected"
    gl.wifi["home"] = False
    gl.log["file"] = open_path
    gl.log["created"] = True
    gl.log["points"] = 3
    gl.log["file_at"] = _clock[0] - gl.NEW_FILE_GAP * 1000 - 1
    gl.log["batch"] = []
    gl.log["prev"] = (52.5, 13.4)
    gl.log["last_t"] = "2026-10-05T18:00:00Z"
    gl.st.update(valid=True, date="2026-10-05", time="18:00:00",
                 lat=52.5, lon=13.4, alt=10.0, pos_at=_clock[0],
                 hdop=1.0, hdop_at=_clock[0], status="A", quality=1,
                 fixtype=3)
    gl.log["fix"] = True
    gl.log["last_sample"] = _clock[0]
    gl.log_tick()
    check("rotate: hotspot file older than 5 min is closed so it can upload",
          gl.log["file"] is None,
          "file=%s" % gl.log.get("file"))
    check("rotate: fix/prev kept (parking does not reopen a file)",
          gl.log["fix"] is True and gl.log["prev"] == (52.5, 13.4))

    gl.log["file"] = open_path
    gl.log["created"] = True
    gl.log["file_at"] = _clock[0] - gl.NEW_FILE_GAP * 1000 - 1
    gl.log["points"] = 3
    gl.wifi["state"] = "offline"
    gl.wifi["home"] = False
    gl.log_tick()
    check("rotate: offline does not close an open file",
          gl.log["file"] == open_path)

    gl.wifi["state"] = "connected"
    gl.wifi["home"] = True
    gl.log["file_at"] = _clock[0] - gl.NEW_FILE_GAP * 1000 - 1
    gl.log_tick()
    check("rotate: home net does not age-rotate (mode_tick closes instead)",
          gl.log["file"] == open_path)

    # failed flush must keep the batch (no drop of up to 60 s of points)
    gl.wifi["state"] = "connected"
    gl.wifi["home"] = False
    gl.log["file"] = open_path
    gl.log["created"] = True
    gl.log["file_at"] = _clock[0] - gl.NEW_FILE_GAP * 1000 - 1
    keep_pt = '<trkpt lat="52.5" lon="13.4"><time>t</time></trkpt>\n'
    gl.log["batch"] = [keep_pt]
    gl.log["err"] = 0
    import builtins
    orig_open = builtins.open

    def _fail_open(*a, **k):
        raise OSError("flash")

    builtins.open = _fail_open
    try:
        gl.log_tick()
    finally:
        builtins.open = orig_open
    check("rotate: failed write keeps the batch and the file",
          gl.log["batch"] == [keep_pt] and gl.log["file"] == open_path
          and gl.log["err"] >= 1,
          "batch=%s file=%s err=%s" % (
              gl.log["batch"], gl.log.get("file"), gl.log.get("err")))

    # ROT-1: 1 h offline driving → one file
    drive_dir = os.path.join(tmpdir, "drive")
    os.mkdir(drive_dir)
    gl.LOG_DIR = drive_dir
    gl.log_reset()
    gl.log["online"] = False
    gl.wifi["state"] = "offline"
    gl.wifi["home"] = False
    gl.log["file"] = None
    seen = set()
    for i in range(3600):
        _clock[0] += 1000
        gl.st.update(valid=True, date="2026-10-05",
                     time="%02d:%02d:%02d" % (i // 3600, (i // 60) % 60, i % 60),
                     lat=52.5 + i * 0.0001, lon=13.4, alt=10.0,
                     pos_at=_clock[0], hdop=1.0, hdop_at=_clock[0],
                     status="A", quality=1, fixtype=3)
        gl.log_tick()
        if gl.log["file"]:
            seen.add(gl.log["file"])
    gl.flush()
    disk = [n for n in os.listdir(drive_dir) if n.endswith(".gpx")]
    check("ROT-1: 1 h offline driving is one file",
          len(seen) == 1 and len(disk) == 1,
          "seen=%s disk=%s points=%s" % (seen, disk, gl.log.get("points")))

    # ROT-2: parking (MIN_MOVE) produces no new files
    park_dir = os.path.join(tmpdir, "park")
    os.mkdir(park_dir)
    gl.LOG_DIR = park_dir
    gl.log_reset()
    gl.log["online"] = False
    gl.wifi["state"] = "offline"
    gl.wifi["home"] = False
    gl.log["prev"] = (52.5, 13.4)
    gl.log["last_t"] = "2026-10-05T00:00:00Z"
    gl.log["file"] = None
    gl.log["fix"] = True
    for i in range(3600):
        _clock[0] += 1000
        gl.st.update(valid=True, date="2026-10-05",
                     time="%02d:%02d:%02d" % (i // 3600, (i // 60) % 60, i % 60),
                     lat=52.5, lon=13.4, alt=10.0, pos_at=_clock[0],
                     hdop=1.0, hdop_at=_clock[0], status="A", quality=1,
                     fixtype=3)
        gl.log_tick()
    gl.flush()
    park_disk = [n for n in os.listdir(park_dir) if n.endswith(".gpx")]
    check("ROT-2: 1 h parked offline opens no file",
          gl.log["file"] is None and park_disk == [],
          "file=%s disk=%s" % (gl.log.get("file"), park_disk))

    # hotspot parking: first recorded point may open one file; age-rotate
    # closes it; MIN_MOVE must not open another
    hot_dir = os.path.join(tmpdir, "hotpark")
    os.mkdir(hot_dir)
    gl.LOG_DIR = hot_dir
    gl.log_reset()
    gl.log["online"] = False
    gl.wifi["state"] = "connected"
    gl.wifi["home"] = False
    gl.log["file"] = None
    gl.log["fix"] = False
    gl.log["prev"] = None
    opened = set()
    for i in range(3600):
        _clock[0] += 1000
        gl.st.update(valid=True, date="2026-10-05",
                     time="%02d:%02d:%02d" % (i // 3600, (i // 60) % 60, i % 60),
                     lat=52.5, lon=13.4, alt=10.0, pos_at=_clock[0],
                     hdop=1.0, hdop_at=_clock[0], status="A", quality=1,
                     fixtype=3)
        gl.log_tick()
        if gl.log["file"]:
            opened.add(gl.log["file"])
    gl.flush()
    hot_disk = [n for n in os.listdir(hot_dir) if n.endswith(".gpx")]
    check("ROT-2: hotspot parking does not keep opening files",
          len(opened) <= 1 and len(hot_disk) <= 1,
          "opened=%s disk=%s file=%s" % (opened, hot_disk, gl.log.get("file")))

    # --- boot-fail is not cleared on import; WDT counts; _boot_ok clears --
    RTC.mem = b"2"
    check("K2: _boot_ok is not run at import",
          gl._boot_ok_done is False)
    gl._boot_ok()
    check("K2: _boot_ok clears RTC memory",
          RTC.mem.startswith(b"0"))
    RTC.mem = b"3"
    gl._boot_ok_done = False
    n = gl._boot_fail_bump()
    check("K2: run-crash bump increments the counter",
          n == 4, "n=%s" % n)

    _reset_cause[0] = 3  # WDT_RESET
    RTC.mem = b"4"
    boot_path = os.path.join(ROOT, "main.py")
    boot_src = open(boot_path).read()
    boot_ns = {
        "Pin": Pin, "RTC": RTC, "machine": sys.modules["machine"],
        "time": time, "__name__": "boot_helpers",
    }
    exec(compile(boot_src, boot_path, "exec"), boot_ns)
    boot_ns["_boot_note_wdt"]()
    check("K2: WDT reset_cause increments boot_fail",
          boot_ns["_boot_fail_get"]() >= 5)

    _reset_cause[0] = 1
    RTC.mem = b"2"
    seen = []
    fake = types.ModuleType("gps_logger")

    def fake_run():
        seen.append(boot_ns["_boot_fail_get"]())
        raise RuntimeError("init crash")

    fake.run = fake_run
    prev = sys.modules.get("gps_logger")
    sys.modules["gps_logger"] = fake
    try:
        boot_ns["_boot_start"]()
    except SystemExit:
        pass
    finally:
        if prev is not None:
            sys.modules["gps_logger"] = prev
        else:
            sys.modules.pop("gps_logger", None)
    check("K2: import does not reset the boot-fail counter",
          seen and seen[0] == 2,
          "seen=%s rtc=%s" % (seen, RTC.mem))
    _reset_cause[0] = 1

    # K2-6: BOOT_OK_MS is counted from the first _loop iteration, not run()
    src = open(os.path.join(ROOT, "gps_logger.py")).read()
    check("K2-6: _loop measures BOOT_OK from loop_start",
          "loop_start = time.ticks_ms()" in src
          and "ticks_diff(now, loop_start)" in src)
    check("ticks: wifi lost last_check uses ticks_add",
          'ticks_add(now, -wifi["backoff"])' in src)
    gl.reset()
    gl.log_reset()
    gl.wifi_reset()
    gl.up_reset()
    gl.wifi["wlan"] = None
    gl.st["start"] = _clock[0] - gl.BOOT_OK_MS - 5000
    RTC.mem = b"3"
    gl._boot_ok_done = False
    orig_sleep = time.sleep_ms
    loops = [0]

    def _one_short(ms):
        _clock[0] += 20
        loops[0] += 1
        raise KeyboardInterrupt()

    time.sleep_ms = _one_short
    try:
        gl._loop()
    except KeyboardInterrupt:
        pass
    finally:
        time.sleep_ms = orig_sleep
    check("K2-6: first _loop iter does not clear even if run() started long ago",
          RTC.mem.startswith(b"3") and gl._boot_ok_done is False,
          "rtc=%s done=%s" % (RTC.mem, gl._boot_ok_done))

    RTC.mem = b"3"
    gl._boot_ok_done = False
    loops[0] = 0

    def _advance_ok(ms):
        _clock[0] += gl.BOOT_OK_MS
        loops[0] += 1
        if loops[0] >= 2:
            raise KeyboardInterrupt()

    time.sleep_ms = _advance_ok
    try:
        gl._loop()
    except KeyboardInterrupt:
        pass
    finally:
        time.sleep_ms = orig_sleep
    check("K2-6: BOOT_OK_MS of _loop clears the boot-fail counter",
          RTC.mem.startswith(b"0") and gl._boot_ok_done is True,
          "rtc=%s done=%s loops=%s" % (RTC.mem, gl._boot_ok_done, loops[0]))

    # restore logger after fake import swap
    gl = _reload_logger() if "gps_logger" in sys.modules else __import__("gps_logger")

finally:
    shutil.rmtree(tmpdir, ignore_errors=True)

# Optional: real MicroPython 1.29 unix port (API-gap catch).
mpy_bin = os.environ.get("MICROPYTHON")
if not mpy_bin:
    for cand in (
            "/tmp/micropython/ports/unix/build-standard/micropython",
            os.path.join(ROOT, "micropython")):
        if os.path.isfile(cand) and os.access(cand, os.X_OK):
            mpy_bin = cand
            break
if mpy_bin:
    import subprocess
    script = os.path.join(ROOT, "tests", "mpy_unix_check.py")
    try:
        out = subprocess.check_output([mpy_bin, script],
                                      stderr=subprocess.STDOUT, timeout=20)
        text = out.decode() if isinstance(out, bytes) else out
        print(text.rstrip())
        check("mpy-unix: v1.29 has no str.isalnum",
              "isalnum False" in text or "isalnum False" in text.replace("=", " "))
        check("mpy-unix: safe_name helpers passed",
              "mpy unix check passed" in text)
    except Exception as e:
        check("mpy-unix: ran MicroPython unix port", False, repr(e))
else:
    print("SKIP  mpy-unix (no micropython binary)")

print()
print("%d passed, %d failed" % (passed, failed))
if failures:
    print("failures:")
    for f in failures:
        print(" ", f)
sys.exit(1 if failed else 0)
