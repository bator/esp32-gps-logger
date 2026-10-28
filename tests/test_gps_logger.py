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
_uart_tx = []
_uart_rx = bytearray()
_uart_ack = [True]
_uart_drop = [0]
_uart_nak = set()
_uart_silent = set()
_freqs = [240000000]


_pin_levels = []


class Pin(object):
    IN = 1
    PULL_UP = 2
    OUT = 3
    fail = ()                                   # pin numbers that raise

    def __init__(self, n, *a, **k):
        if n in Pin.fail:
            raise ValueError("invalid pin")
        self.n = n
        self._v = 0

    def value(self, v=None):
        if v is None:
            return 1
        self._v = v
        _pin_levels.append((self.n, v, _clock[0]))
        return None


class I2C(object):
    def __init__(self, *a, **k):
        pass


class UART(object):
    def __init__(self, *a, **k):
        pass

    def any(self):
        return len(_uart_rx)

    def read(self, n=None):
        n = len(_uart_rx) if n is None else min(n, len(_uart_rx))
        out = bytes(_uart_rx[:n])
        del _uart_rx[:n]
        return out

    def readinto(self, buf):
        out = self.read(len(buf))
        buf[:len(out)] = out
        return len(out)

    def write(self, data):
        data = bytes(data)
        _uart_tx.append(data)
        # a u-blox acknowledges a UBX-CFG frame (unless told to drop it)
        if _uart_ack[0] and data[:3] == b"\xb5\x62\x06":
            if _uart_drop[0]:
                _uart_drop[0] -= 1
            elif (data[2], data[3]) in _uart_silent:
                pass
            else:
                kind = 0x00 if (data[2], data[3]) in _uart_nak else 0x01
                ack = bytes((0x05, kind, 2, 0, data[2], data[3]))
                ca = cb = 0
                for x in ack:
                    ca = (ca + x) & 255
                    cb = (cb + ca) & 255
                _uart_rx.extend(b"\xb5\x62" + ack + bytes((ca, cb)))
        return len(data)


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
    assoc_channel = 1
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
            ch = item[3] if len(item) > 3 else 1
            out.append((ssid.encode(), bssid, ch, rssi, 4, False))
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

    ifconfig_calls = []

    def ifconfig(self, cfg=None):
        # v1.29 esp32: a 4-tuple sets a static address, "dhcp" restores DHCP
        if cfg is not None:
            WLAN.ifconfig_calls.append(cfg)
            return None
        return ("192.168.1.8", "255.255.255.0", "192.168.1.1", "1.1.1.1")

    def config(self, *a, **k):
        # v1.29 network_wlan.c: unknown keys raise ValueError. There is
        # no 'bssid' getter; 'channel' is supported.
        if a and a[0] == "bssid":
            raise ValueError("unknown config param")
        if a and a[0] == "channel":
            return WLAN.assoc_channel
        return None


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

    def _freq(v=None):
        if v is None:
            return _freqs[-1]
        if v not in (20000000, 40000000, 80000000, 160000000, 240000000):
            raise ValueError("bad freq")
        _freqs.append(v)
        return None
    machine.freq = _freq
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


diag_dir = tempfile.mkdtemp(prefix="diaglog-")


def _reload_logger():
    if "gps_logger" in sys.modules:
        m = importlib.reload(sys.modules["gps_logger"])
    else:
        import gps_logger as m
    m.DIAG_FILE = os.path.join(diag_dir, "diag.log")   # never the host's /
    m.DIAG_OLD = os.path.join(diag_dir, "diag.1")
    m.WEB = False                        # radio-sleep tests; WEB has its own
    return m


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


GPX_HEADER = ('<?xml version="1.0" encoding="UTF-8"?>\n'
              '<gpx version="1.1" creator="esp32 gps_logger" '
              'xmlns="http://www.topografix.com/GPX/1/1">\n'
              '<trk><name>%s</name><trkseg>\n')
GPX_FOOTER = "</trkseg></trk></gpx>\n"


def make_gpx(path, n, extra=b"", times=None, extras=None):
    """Write n points to path. A .trk path gets the compact format, anything
    else the legacy GPX XML (old .gpx files, still uploaded).
    extra is appended raw before the footer; extras holds per-point XML tags
    for GPX only."""
    if path.endswith(".trk"):
        return make_trk(path, n, extra=extra, times=times)
    name = os.path.basename(path)[:-4]
    parts = [(GPX_HEADER % name).encode()]
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
    parts.append(GPX_FOOTER.encode())
    with open(path, "wb") as f:
        f.write(b"".join(parts))


def trk_line(lat, lon, t, ele="10.0", hdop=""):
    """One .trk point line; t is the ISO form, stored compact."""
    tc = t.replace("-", "").replace(":", "")
    return "%.6f,%.6f,%s,%s,%s\n" % (lat, lon, ele, hdop, tc)


def pts_in(path):
    """Point lines in a track file (.trk) or a legacy .gpx file."""
    gpx = not gl._is_trk(path)
    with open(path, "rb") as f:
        return sum(1 for line in f if gl._is_point(line, gpx))


def make_trk(path, n, extra=b"", times=None):
    parts = [gl.HEADER.encode()]
    for i in range(n):
        t = (times[i] if times else
             "2026-10-05T%02d:%02d:%02dZ" % (i // 3600, (i // 60) % 60, i % 60))
        parts.append(trk_line(52.5 + i * 0.0001, 13.4 + i * 0.0001, t).encode())
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
            if n.endswith((".gpx", ".trk", ".pos", ".tmp", ".bad")):
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
    gl.log["full"] = True
    gl.wifi["state"] = "lost"
    check("full: losing WiFi does not clear the full-flash state",
          gl.log["full"] is True)
    gl.log["file"] = os.path.join(tmpdir, "20261005_160000.gpx")
    gl.log["created"] = False
    gl.log["batch"] = ['<trkpt lat="1" lon="2"><time>t</time></trkpt>\n']
    orig_free = gl.free
    gl.free = lambda: 100
    gl.flush()
    check("full: flush() with MIN_FREE not met sets full and drops batch",
          gl.log["full"] is True and gl.log["batch"] == [])
    gl.free = lambda: gl.MIN_FREE + 10
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
    gl.log["file"] = os.path.join(tmpdir, "now.trk")
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
    check("HDOP: recorded line stores hdop",
          gl.log["batch"][0].split(",")[3] == "1.2",
          "line=%r" % gl.log["batch"][0])
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

    # static IP is per network; DHCP networks are not touched
    static_env = (triple + "IP=192.168.1.60\nGW=192.168.1.1\n"
                  "DNS=192.168.1.111\n")
    nets = gl._load_nets(dict(l.split("=", 1) for l in static_env.splitlines()))
    check("static ip: #1 parsed with mask default and given DNS",
          nets[0]["ip"] == ("192.168.1.60", "255.255.255.0", "192.168.1.1",
                            "192.168.1.111"), str(nets[0]["ip"]))
    check("static ip: #2 and #3 stay DHCP",
          nets[1]["ip"] is None and nets[2]["ip"] is None)
    nets = gl._load_nets({"SSID": "a", "IP": "192.168.1.60"})
    check("static ip: missing gateway falls back to DHCP", nets[0]["ip"] is None)
    nets = gl._load_nets({"SSID": "a", "IP": "192.168.1.300", "GW": "192.168.1.1"})
    check("static ip: invalid address falls back to DHCP", nets[0]["ip"] is None)
    nets = gl._load_nets({"SSID": "a", "IP": "192.168.1.60", "GW": "192.168.1.1",
                          "MASK": "255.255.0.0"})
    check("static ip: MASK given, DNS defaults to the gateway",
          nets[0]["ip"] == ("192.168.1.60", "255.255.0.0", "192.168.1.1",
                            "192.168.1.1"), str(nets[0]["ip"]))

    WLAN.ifconfig_calls = []
    w = start_wifi(static_env, [("home", -40)], ["home"])
    check("static ip: #1 connects with the static address applied",
          w["state"] == "connected"
          and WLAN.ifconfig_calls == [("192.168.1.60", "255.255.255.0",
                                       "192.168.1.1", "192.168.1.111")],
          str(WLAN.ifconfig_calls))
    WLAN.ifconfig_calls = []
    w = start_wifi(single, [("home", -40)], ["home"])
    check("static ip: a DHCP-only config never calls ifconfig",
          WLAN.ifconfig_calls == [], str(WLAN.ifconfig_calls))
    gl.wifi["static"] = True
    WLAN.ifconfig_calls = []
    gl._wifi_ip(WLAN(0), {"ssid": "phone", "ip": None})
    check("static ip: leaving the static net hands the interface back to DHCP",
          WLAN.ifconfig_calls == ["dhcp"] and not gl.wifi["static"],
          str(WLAN.ifconfig_calls))

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
    env_hid_home = (triple + "SSID_HIDDEN=1\n")
    w = start_wifi(env_hid_home, [("phone", -40)], ["phone"])
    check("home-first: hidden home not tried before a visible hotspot",
          WLAN.connects and WLAN.connects[0][0] == "phone"
          and w["ssid"] == "phone",
          "connects=%s ssid=%s" % (WLAN.connects, w.get("ssid")))
    w = start_wifi(env_hid_home, [("", -55), ("phone", -40)], ["home", "phone"])
    check("HID: strong empty-SSID AP prefers hidden home at boot",
          WLAN.connects and WLAN.connects[0][0] == "home"
          and w["ssid"] == "home",
          "connects=%s ssid=%s" % (WLAN.connects, w.get("ssid")))
    w = start_wifi(env_hid_home, [("", -80), ("phone", -40)], ["home", "phone"])
    check("HID: weak empty-SSID AP does not prefer hidden home",
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

    w = start_wifi(triple, [("unrelated", -40)], [])
    check("wifi all absent -> offline (not connected)",
          w["state"] != "connected")
    check("wifi scan missing known SSIDs: configured nets are fallback candidates",
          [c[0] for c in WLAN.connects] == ["home", "phone", "cabin"],
          "connects=%s" % (WLAN.connects,))
    w = start_wifi(triple, [("unrelated", -40)], ["phone"])
    check("wifi scan misses saved SSIDs: #2 still connects",
          w["state"] == "connected" and w["ssid"] == "phone",
          "state=%s ssid=%s connects=%s" % (
              w["state"], w["ssid"], WLAN.connects))
    # logging continues
    gl.log_reset()
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
    _post_code[0] = 201
    gl.up["hold"] = {}
    n_pump = 0
    while os.path.exists(path) and n_pump < 6:
        gl.up["next_at"] = _clock[0]
        pump()
        n_pump += 1
    check("B1: BAD FULL clears after the stuck file is gone",
          not gl._bad_full_left() and gl._mode_line() != "Mode BAD FULL",
          "state=%s mode=%s exists=%s" % (
              gl.up["state"], gl._mode_line(), os.path.exists(path)))

    # --- B2: 200 kB line without newline must not MemoryError/boot-loop ----
    online(tmpdir)
    path = os.path.join(tmpdir, "20261005_20long.gpx")
    with open(path, "wb") as f:
        f.write((GPX_HEADER % "20261005_20long").encode())
        f.write(b'<trkpt lat="52.500000" lon="13.400000">'
                b'<time>2026-10-05T00:00:00Z</time></trkpt>\n')
        f.write(b"A" * 200000 + b"\n")
        f.write(b'<trkpt lat="52.500100" lon="13.400100">'
                b'<time>2026-10-05T00:00:01Z</time></trkpt>\n')
        f.write(GPX_FOOTER.encode())
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
    def gps_fix(lat=52.5, speed=6.0):
        gl.st.update(valid=True, date="2026-10-05", time="12:00:00",
                     lat=lat, lon=13.4, alt=10.0, pos_at=_clock[0],
                     hdop=1.0, hdop_at=_clock[0], status="A", quality=1,
                     fixtype=3, speed=speed)

    env_hot = ("SSID=home\nWPA2=h\nSSID2=PhoneHotspot\nWPA2_2=p\n"
               "GEO_URL=https://d.example\nAPI_KEY=k\n")
    w = start_wifi(env_hot, [("PhoneHotspot", -40)], ["PhoneHotspot"])
    check("B3: connected to non-home hotspot",
          w["state"] == "connected" and w["ssid"] == "PhoneHotspot"
          and w["home"] is False)
    check("B3: at_home() is False on hotspot", gl.at_home() is False)
    gl.log_reset()
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
        if n.endswith((".gpx", ".trk", ".pos", ".tmp", ".bad")):
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
    gps_fix(52.51, speed=0.0)
    gl.log["last_sample"] = None
    gl.log_tick()
    home_path = gl.log["file"]
    check("B3: home WiFi records GPS points",
          bool(gl.log["batch"]) and home_path is not None)
    check("B3: OLED mode LOG+UP on home",
          gl._mode_line() == "Mode LOG+UP")

    # HOME STAY: two calm minutes on the home network close the track,
    # whatever the (noisy) GPS speed says; a departure opens a new one
    import random
    hs_rnd = random.Random(7)
    hs_t = [0]

    def hs_fix(lat, speed):
        _clock[0] += gl.SAMPLE
        hs_t[0] += 1
        gps_fix(lat, speed=speed)
        gl.st["time"] = "%02d:%02d:%02d" % (12 + hs_t[0] // 3600,
                                            (hs_t[0] // 60) % 60, hs_t[0] % 60)
        gl.log_tick()

    for _ in range(gl.STILL_WINDOW // 1000):
        hs_fix(52.51 + hs_rnd.uniform(-1e-4, 1e-4), hs_rnd.uniform(0, 22))
    check("HOME STAY: one calm minute is not a stay yet",
          gl.log["still"]["active"] is False and gl.log["file"] == home_path)
    for _ in range(gl.STILL_WINDOW // 1000 + 2):
        hs_fix(52.51 + hs_rnd.uniform(-1e-4, 1e-4), hs_rnd.uniform(0, 22))
    # the drift that proves the stay is never uploaded (all of it was held
    # in RAM), so the track it opened is never even created
    check("HOME STAY: two calm minutes close the track despite speed noise",
          gl.log["still"]["active"] is True and gl.log["still"]["home"] is True
          and gl.log["file"] is None and gl.log["batch"] == []
          and home_path not in gl.pending(),
          "still=%s file=%s" % (gl.log["still"]["active"], gl.log["file"]))
    n_files = len(gl.pending())
    for _ in range(20 * 60):
        hs_fix(52.51 + hs_rnd.uniform(-3e-4, 3e-4), hs_rnd.uniform(0, 22))
    check("HOME STAY: 20 min of drift at home writes nothing",
          gl.log["file"] is None and gl.log["batch"] == []
          and len(gl.pending()) == n_files and gl.log["still"]["active"])
    WLAN.scan_aps = [("home", -90)]                 # home AP out of range
    for i in range(1, 40):                          # drive away north, 6 m/s
        hs_fix(52.51 + i * 5.4e-5, 20.0)
    resumed_path = gl.log["file"]
    first = gl.log["batch"][0].split(",") if gl.log["batch"] else None
    check("HOME STAY: driving away opens a new track from the anchor",
          gl.log["still"]["active"] is False and resumed_path is not None
          and first is not None and abs(float(first[0]) - 52.51) < 3e-4,
          "file=%s first=%r" % (resumed_path, first))

    gl.wifi["home"] = False
    _clock[0] += 30000
    gl.st["pos_at"] = _clock[0]
    gl.st["hdop_at"] = _clock[0]
    gl.log_tick()
    _clock[0] += gl.NEW_FILE_GAP * 1000
    gl.st["time"] = "13:30:01"
    gl.st["lat"] = 52.52
    gl.st["speed"] = 6.0
    gl.st["pos_at"] = _clock[0]
    gl.st["hdop_at"] = _clock[0]
    gl.log["last_sample"] = None
    gl.log_tick()
    check("B3: connected WiFi closes old tracks for background upload",
          resumed_path in gl.pending()
          and gl.log["file"] != resumed_path,
          "pending=%s current=%s" % (gl.pending(), gl.log.get("file")))

    # HOME2 override
    env_h2 = env_hot + "HOME2=1\n"
    w = start_wifi(env_h2, [("PhoneHotspot", -40)], ["PhoneHotspot"])
    check("B3: HOME2=1 treats hotspot as home",
          w["home"] is True, "home=%s" % w["home"])

    # --- TRK: compact track format (.trk) ----------------------------------
    saved_dir = gl.LOG_DIR
    trk_dir = os.path.join(tmpdir, "trkfmt")
    os.makedirs(trk_dir)
    gl.LOG_DIR = trk_dir
    gl.wifi["state"] = "offline"
    gl.log_reset()
    gl.up.setdefault("left", 0)
    gl.st.update(valid=True, date="2026-10-05", time="12:00:05", lat=52.5,
                 lon=13.4, alt=10.0, pos_at=_clock[0], hdop=1.2,
                 hdop_at=_clock[0], status="A", quality=1, fixtype=3)
    gl.log_tick()
    rec = gl.log["batch"][0] if gl.log["batch"] else ""
    check("TRK-1: recorded point is lat,lon,ele,hdop,YYYYMMDDTHHMMSSZ",
          rec == "52.500000,13.400000,10.0,1.2,20261005T120005Z\n", repr(rec))
    check("TRK-1: new file is a .trk under LOG_DIR",
          gl.log["file"] is not None and gl.log["file"].endswith(".trk")
          and os.path.dirname(gl.log["file"]) == trk_dir,
          "file=%s" % gl.log.get("file"))

    gl.log_reset()
    gl.st.update(fixtype=2, time="12:00:06")
    gl.log_tick()
    check("TRK-2: 2D fix leaves ele empty",
          gl.log["batch"][0:1] == ["52.500000,13.400000,,1.2,20261005T120006Z\n"],
          repr(gl.log["batch"]))

    gl.log_reset()
    f1 = os.path.join(trk_dir, "flush.trk")
    p1 = trk_line(52.5, 13.4, "2026-10-05T12:00:05Z")
    p2 = trk_line(52.6, 13.5, "2026-10-05T12:00:06Z")
    p3 = trk_line(52.7, 13.6, "2026-10-05T12:00:07Z")
    gl.log.update(file=f1, batch=[p1, p2])
    gl.flush()
    gl.log["batch"] = [p3]
    gl.flush()
    with open(f1, "rb") as f:
        raw = f.read()
    check("TRK-3: flush writes HEADER, point lines, FOOTER; appends keep the footer last",
          raw == (gl.HEADER + p1 + p2 + p3 + gl.FOOTER).encode(), repr(raw))

    rb = os.path.join(trk_dir, "read.trk")
    with open(rb, "wb") as f:
        f.write((gl.HEADER
                 + "52.500000,13.400000,10.0,1.2,20261005T120005Z\n"
                 + "-\n"
                 + "-33.800000,-70.600000,,,20261005T120010Z\n"
                 + "52.500000,13.400000,10.0,1.0,20261005X120005Z\n"
                 + "52.500000,13.400000,10.0,1.0,2026100").encode())
    n, n_ok, size, new_off, eof = gl.read_batch(rb, 0)
    check("TRK-4: read_batch counts point lines, skips bad time and the cut-off tail",
          n == 3 and n_ok == 2 and eof is True,
          "n=%s n_ok=%s eof=%s" % (n, n_ok, eof))
    locs = json.loads(bytes(gl._body[:size]).decode())["locations"]
    check("TRK-4: Overland JSON from .trk has ISO time, altitude, accuracy, negative coords",
          len(locs) == 2
          and locs[0]["properties"]["timestamp"] == "2026-10-05T12:00:05Z"
          and locs[0]["properties"]["altitude"] == 10.0
          and abs(locs[0]["properties"]["horizontal_accuracy"] - 6.0) < 1e-6
          and locs[1]["geometry"]["coordinates"] == [-70.6, -33.8]
          and "altitude" not in locs[1]["properties"],
          json.dumps(locs)[:200])

    cnt_dir = os.path.join(tmpdir, "trkcount")
    os.makedirs(cnt_dir)
    gl.LOG_DIR = cnt_dir
    shutil.copy(rb, os.path.join(cnt_dir, "mix.trk"))
    old_gpx = os.path.join(cnt_dir, "old.gpx")
    make_gpx(old_gpx, 3)
    total = gl.count_points()
    check("TRK-5: count_points counts .trk and legacy .gpx points only",
          total == 3 + 3, "total=%s" % total)
    n_g, n_ok_g, _, _, _ = gl.read_batch(old_gpx, 0)
    check("TRK-5: legacy .gpx is still read for upload",
          n_g == 3 and n_ok_g == 3, "n=%s n_ok=%s" % (n_g, n_ok_g))

    check("TRK-6: a compact point line is under 50 bytes (old GPX point was 112)",
          len(trk_line(52.5, 13.4, "2026-10-05T12:00:05Z", hdop="1.2")
              .encode()) <= 50)

    qsrc = os.path.join(trk_dir, "q.trk")
    make_trk(qsrc, 3)
    _, _, _, qnew, _ = gl.read_batch(qsrc, 0)
    ok_bad = gl._append_bad(qsrc, 0, qnew)
    with open(qsrc + ".bad", "rb") as f:
        bad = f.read()
    check("TRK-7: quarantine copies only the point lines of a .trk",
          ok_bad and bad.count(b"\n") == 3 and b"#" not in bad,
          "ok=%s bad=%r" % (ok_bad, bad[:80]))

    gl.LOG_DIR = saved_dir

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
    check("K6: empty scan falls back through configured networks",
          [c[0] for c in WLAN.connects] == ["home", "PhoneHotspot"],
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

    w = start_wifi(env_hid_home, [("phone", -40)], ["phone"])
    check("HID roam setup: on hotspot",
          w["ssid"] == "phone" and w["home"] is False)
    WLAN.scan_aps = [("", -55), ("phone", -40)]
    WLAN.ok = set(["home", "phone"])
    WLAN.connects = []
    gl.st["speed"] = None
    gl.wifi["failed"].pop("home", None)
    gl.wifi["roam_at"] = _clock[0] - gl.ROAM_CHECK
    gl.wifi_tick()
    check("HID: roam tries hidden home when empty-SSID AP is strong",
          WLAN.connects and WLAN.connects[0][0] == "home",
          "connects=%s" % (WLAN.connects,))
    WLAN.ok = set(["phone"])
    drain_wifi()
    WLAN.scan_aps = [("phone", -40)]
    WLAN.connects = []
    if WLAN.instances:
        WLAN.instances[-1]._status = 3
    gl.wifi["state"] = "connected"
    gl.wifi["home"] = False
    gl.wifi["ssid"] = "phone"
    gl.wifi["roam_at"] = _clock[0] - gl.ROAM_CHECK
    gl.wifi_tick()
    check("HID: no empty-SSID AP does not roam to hidden home",
          WLAN.connects == [],
          "connects=%s" % (WLAN.connects,))

    # stronger neighbouring hidden AP must not steal the roam BSSID
    w = start_wifi(env_hid_home, [("phone", -40)], ["phone"])
    check("HID neighbour setup: on hotspot",
          w["ssid"] == "phone" and w["home"] is False)
    neighbour = b"\x0a" * 6
    home_b = b"\x01" * 6
    WLAN.scan_aps = [("", -40, neighbour), ("", -65, home_b), ("phone", -40)]
    WLAN.ok = set(["home", "phone"])
    WLAN.connects = []
    gl.st["speed"] = None
    gl.wifi["failed"].pop("home", None)
    gl.wifi["roam_at"] = _clock[0] - gl.ROAM_CHECK
    gl.wifi_tick()
    check("HID: roam does not pin to a stronger neighbour hidden AP",
          WLAN.connects and WLAN.connects[0][0] == "home"
          and WLAN.connects[0][2] is None,
          "connects=%s" % (WLAN.connects,))
    WLAN.ok = set(["phone"])
    drain_wifi()
    if WLAN.instances:
        WLAN.instances[-1]._status = 3
    gl.wifi["state"] = "connected"
    gl.wifi["home"] = False
    gl.wifi["ssid"] = "phone"
    gl.wifi["roam_backoff"] = gl.ROAM_BACKOFF_MAX
    gl.wifi["moved_at"] = _clock[0] - 60000
    gl.wifi["roam_at"] = _clock[0]
    gl.wifi["failed"].pop("home", None)
    gl.st.update(valid=True, date="2026-10-05", time="18:00:00",
                 lat=52.5, lon=13.4, pos_at=_clock[0], speed=0,
                 status="A", quality=1, fixtype=3)
    WLAN.connects = []
    gl.wifi_tick()
    check("HID: stop after driving does not reset backoff on empty-SSID-only",
          (gl.wifi.get("roam_backoff") or 0) == gl.ROAM_BACKOFF_MAX
          and WLAN.connects == [],
          "backoff=%s connects=%s" % (
              gl.wifi.get("roam_backoff"), WLAN.connects))
    gl.st["speed"] = None

    raised = False
    try:
        WLAN(0).config("bssid")
    except ValueError:
        raised = True
    check("HID: mock config('bssid') raises ValueError (v1.29)",
          raised)

    w = start_wifi(env_hid_home, [("phone", -40)], ["phone"])
    WLAN.scan_aps = [("", -55, neighbour), ("phone", -40)]
    WLAN.ok = set(["phone"])
    WLAN.connects = []
    gl.st["speed"] = None
    gl.wifi["failed"].pop("home", None)
    gl.wifi["roam_backoff"] = gl.ROAM_CHECK
    gl.wifi["roam_at"] = _clock[0] - gl.ROAM_CHECK
    gl.wifi_tick()
    drain_wifi()
    check("HID: failed hidden-home roam keeps the 5 min interval",
          (gl.wifi.get("roam_backoff") or 0) == gl.ROAM_CHECK,
          "backoff=%s" % gl.wifi.get("roam_backoff"))
    if WLAN.instances:
        WLAN.instances[-1]._status = 3
    gl.wifi["state"] = "connected"
    gl.wifi["home"] = False
    gl.wifi["ssid"] = "phone"
    for _ in range(5):
        _clock[0] += gl.FAIL_HOLD
        gl.wifi["failed"].pop("home", None)
        gl.wifi["roam_at"] = _clock[0] - gl.ROAM_CHECK
        if WLAN.instances:
            WLAN.instances[-1]._status = 3
        gl.wifi["state"] = "connected"
        gl.wifi["home"] = False
        gl.wifi["ssid"] = "phone"
        gl.wifi_tick()
        drain_wifi()
    check("HID: repeated hidden-home fails do not grow backoff",
          (gl.wifi.get("roam_backoff") or 0) == gl.ROAM_CHECK,
          "backoff=%s" % gl.wifi.get("roam_backoff"))

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
    _clock[0] += gl.FAIL_HOLD
    gl.wifi["failed"].pop("home", None)
    gl.wifi["roam_at"] = _clock[0] - gl.wifi["roam_backoff"]
    gl.wifi_tick()
    check("roam backoff: retries after the doubled interval",
          WLAN.connects and WLAN.connects[0][0] == "home",
          "connects=%s" % (WLAN.connects,))
    WLAN.ok = set(["PhoneHotspot"])
    drain_wifi()
    if WLAN.instances:
        WLAN.instances[-1]._status = 3
    gl.wifi["state"] = "connected"
    gl.wifi["home"] = False
    gl.wifi["ssid"] = "PhoneHotspot"
    gl.wifi["roam_backoff"] = gl.ROAM_CHECK
    WLAN.scan_aps = [("PhoneHotspot", -40)]
    WLAN.connects = []
    gl.st.update(valid=True, date="2026-10-05", time="18:00:00",
                 lat=52.5, lon=13.4, pos_at=_clock[0], speed=0,
                 status="A", quality=1, fixtype=3)
    for _ in range(6):
        gl.wifi["roam_at"] = _clock[0] - gl.ROAM_CHECK
        gl.wifi_tick()
        drain_wifi()
        if WLAN.instances:
            WLAN.instances[-1]._status = 3
        gl.wifi["state"] = "connected"
        gl.wifi["home"] = False
        gl.wifi["ssid"] = "PhoneHotspot"
    check("roam backoff: unseen home keeps the 5 min interval",
          (gl.wifi.get("roam_backoff") or 0) == gl.ROAM_CHECK
          and WLAN.connects == [],
          "backoff=%s connects=%s" % (
              gl.wifi.get("roam_backoff"), WLAN.connects))
    # long trip then home appears: still due in 5 min, not 45–60
    gl.wifi["failed"].pop("home", None)
    _clock[0] += gl.FAIL_HOLD
    WLAN.scan_aps = [("home", -55), ("PhoneHotspot", -40)]
    WLAN.ok = set(["home", "PhoneHotspot"])
    WLAN.connects = []
    if WLAN.instances:
        WLAN.instances[-1]._status = 3
    gl.wifi["state"] = "connected"
    gl.wifi["home"] = False
    gl.wifi["ssid"] = "PhoneHotspot"
    gl.wifi["roam_at"] = _clock[0] - gl.ROAM_CHECK
    gl.wifi_tick()
    check("roam: after a long hotspot trip, visible home is tried at 5 min",
          WLAN.connects and WLAN.connects[0][0] == "home",
          "connects=%s backoff=%s" % (
              WLAN.connects, gl.wifi.get("roam_backoff")))
    WLAN.ok = set(["PhoneHotspot"])
    drain_wifi()
    gl.st["speed"] = 50
    gl.st["pos_at"] = _clock[0]
    WLAN.scan_aps = [("home", -40), ("PhoneHotspot", -50)]
    WLAN.ok = set(["home", "PhoneHotspot"])
    WLAN.connects = []
    if WLAN.instances:
        WLAN.instances[-1]._status = 3
    gl.wifi["state"] = "connected"
    gl.wifi["home"] = False
    gl.wifi["ssid"] = "PhoneHotspot"
    gl.wifi["roam_backoff"] = gl.ROAM_CHECK
    gl.wifi["roam_at"] = _clock[0] - gl.ROAM_CHECK
    gl.wifi_tick()
    check("roam: skip scan while moving >= 5 km/h",
          WLAN.connects == [],
          "connects=%s" % (WLAN.connects,))
    # stale speed (no fresh fix) must not block
    gl.wifi["failed"].pop("home", None)
    gl.st["pos_at"] = _clock[0] - gl.STALE - 1
    gl.st["speed"] = 50
    WLAN.connects = []
    if WLAN.instances:
        WLAN.instances[-1]._status = 3
    gl.wifi["state"] = "connected"
    gl.wifi["home"] = False
    gl.wifi["ssid"] = "PhoneHotspot"
    gl.wifi["roam_at"] = _clock[0] - gl.ROAM_CHECK
    gl.wifi_tick()
    check("roam: stale speed does not skip the home scan",
          WLAN.connects and WLAN.connects[0][0] == "home",
          "connects=%s" % (WLAN.connects,))
    gl.st["pos_at"] = _clock[0]
    gl.st["speed"] = None
    WLAN.ok = set(["PhoneHotspot"])
    drain_wifi()
    if WLAN.instances:
        WLAN.instances[-1]._status = 3
    gl.wifi["state"] = "connected"
    gl.wifi["home"] = False
    gl.wifi["ssid"] = "PhoneHotspot"
    gl.wifi["failed"].pop("home", None)
    gl.wifi["roam_backoff"] = gl.ROAM_BACKOFF_MAX
    gl.wifi["moved_at"] = _clock[0] - 60000
    gl.st["speed"] = 0
    gl.st["pos_at"] = _clock[0]
    WLAN.connects = []
    if WLAN.instances:
        WLAN.instances[-1]._status = 3
    gl.wifi["state"] = "connected"
    gl.wifi["home"] = False
    gl.wifi["ssid"] = "PhoneHotspot"
    gl.wifi["roam_at"] = _clock[0]
    gl.wifi_tick()
    check("roam: stop after driving resets backoff and scans",
          (gl.wifi.get("roam_backoff") or 0) == gl.ROAM_CHECK
          and WLAN.connects and WLAN.connects[0][0] == "home",
          "backoff=%s connects=%s" % (
              gl.wifi.get("roam_backoff"), WLAN.connects))
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
    open_path = os.path.join(rot_dir, "open.trk")
    make_gpx(open_path, 3)
    gl.log_reset()
    gl.wifi["state"] = "connected"
    gl.wifi["home"] = False
    gl.log["file"] = open_path
    gl.log["created"] = True
    gl.log["points"] = 3
    gl.log["file_at"] = _clock[0] - gl.NEW_FILE_GAP * 1000 - 1
    gl.log["last_flush"] = _clock[0] - gl.FLUSH - 1
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
    gl.log["last_flush"] = _clock[0] - gl.FLUSH - 1
    gl.log["points"] = 3
    gl.wifi["state"] = "offline"
    gl.wifi["home"] = False
    gl.log_tick()
    check("rotate: offline does not close an open file",
          gl.log["file"] == open_path)

    gl.wifi["state"] = "connected"
    gl.wifi["home"] = True
    gl.log["file_at"] = _clock[0] - gl.NEW_FILE_GAP * 1000 - 1
    gl.log["last_flush"] = _clock[0] - gl.FLUSH - 1
    gl.log_tick()
    check("rotate: home net closes an old file so it can upload",
          gl.log["file"] is None)

    # failed flush must keep the batch (no drop of up to 60 s of points)
    gl.wifi["state"] = "connected"
    gl.wifi["home"] = False
    gl.log["file"] = open_path
    gl.log["created"] = True
    gl.log["file_at"] = _clock[0] - gl.NEW_FILE_GAP * 1000 - 1
    gl.log["last_flush"] = _clock[0] - gl.FLUSH - 1
    keep_pt = trk_line(52.5, 13.4, "2026-10-05T00:00:00Z")
    gl.log["batch"] = [keep_pt]
    gl.log["err"] = 0
    import builtins
    orig_open = builtins.open

    def _fail_open(*a, **k):
        raise OSError("flash")

    builtins.open = _fail_open
    try:
        gl.log_tick()
        n0 = len(gl.log["batch"])
        opens = [0]
        real_fail = _fail_open

        def _count_fail(*a, **k):
            opens[0] += 1
            raise OSError("flash")

        builtins.open = _count_fail
        for i in range(30):
            _clock[0] += 1000
            gl.st.update(valid=True, date="2026-10-05",
                         time="18:%02d:%02d" % ((i + 1) // 60, (i + 1) % 60),
                         lat=52.5 + i * 0.0002, lon=13.4, alt=10.0,
                         pos_at=_clock[0], hdop=1.0, hdop_at=_clock[0],
                         status="A", quality=1, fixtype=3)
            gl.log_tick()
    finally:
        builtins.open = orig_open
    check("rotate: failed write keeps the batch and the file",
          keep_pt in gl.log["batch"] and gl.log["file"] == open_path
          and gl.log["err"] >= 1,
          "batch=%s file=%s err=%s" % (
              gl.log["batch"], gl.log.get("file"), gl.log.get("err")))
    check("ROT-8: write errors keep sampling into RAM (retry at FLUSH)",
          len(gl.log["batch"]) > n0 and opens[0] <= 2,
          "batch=%s n0=%s opens=%s" % (
              len(gl.log["batch"]), n0, opens[0]))

    # ROT-1: 1 h offline driving → one file
    drive_dir = os.path.join(tmpdir, "drive")
    os.mkdir(drive_dir)
    gl.LOG_DIR = drive_dir
    gl.log_reset()
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
    disk = [n for n in os.listdir(drive_dir) if n.endswith(".trk")]
    check("ROT-1: 1 h offline driving is one file",
          len(seen) == 1 and len(disk) == 1,
          "seen=%s disk=%s points=%s" % (seen, disk, gl.log.get("points")))

    # ROT-2: parking produces no stream of files
    park_dir = os.path.join(tmpdir, "park")
    os.mkdir(park_dir)
    gl.LOG_DIR = park_dir
    gl.log_reset()
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
    park_disk = [n for n in os.listdir(park_dir) if n.endswith(".trk")]
    park_lines = []
    for n in park_disk:
        with open(os.path.join(park_dir, n)) as f:
            park_lines += [l for l in f if l.count(",") == 4]
    # a stay writes its anchor every STILL_HEARTBEAT, all into one file
    check("ROT-2: 1 h parked offline is one file of anchor points",
          len(park_disk) == 1 and len(park_lines) <= 14
          and all(l.split(",")[2] == "" for l in park_lines),
          "file=%s disk=%s lines=%d" % (gl.log.get("file"), park_disk,
                                         len(park_lines)))

    # hotspot parking: the first point opens a file, the stay closes it and
    # rotates its anchor file every STILL_FILE; MIN_MOVE opens no others
    hot_dir = os.path.join(tmpdir, "hotpark")
    os.mkdir(hot_dir)
    gl.LOG_DIR = hot_dir
    gl.log_reset()
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
    hot_disk = [n for n in os.listdir(hot_dir) if n.endswith(".trk")]
    check("ROT-2: hotspot parking does not keep opening files",
          len(opened) <= 1 + 3600000 // gl.STILL_FILE + 1
          and len(hot_disk) <= 1 + 3600000 // gl.STILL_FILE + 1,
          "opened=%s disk=%s file=%s" % (opened, hot_disk, gl.log.get("file")))

    # --- close() error: drop batch only if the new size actually landed ----
    close_dir = os.path.join(tmpdir, "close")
    os.mkdir(close_dir)
    gl.LOG_DIR = close_dir
    extra_pt = trk_line(52.51, 13.41, "2026-10-05T00:00:02Z")

    def _gpx_pts(path):
        if not os.path.exists(path):
            return 0
        gpx = not gl._is_trk(path)
        with orig_open(path, "rb") as f:
            return sum(1 for line in f if gl._is_point(line, gpx))

    orig_open = builtins.open

    def _close_wrap(rollback):
        def _open(path, *a, **k):
            existed = os.path.exists(path)
            backup = None
            if rollback and existed:
                bf = orig_open(path, "rb")
                try:
                    backup = bf.read()
                finally:
                    bf.close()
            inner = orig_open(path, *a, **k)

            class _W(object):
                def seek(self, *x, **y):
                    return inner.seek(*x, **y)

                def tell(self):
                    return inner.tell()

                def read(self, *x, **y):
                    return inner.read(*x, **y)

                def write(self, *x, **y):
                    return inner.write(*x, **y)

                def close(self):
                    inner.close()
                    if rollback:
                        if not existed:
                            try:
                                os.remove(path)
                            except OSError:
                                pass
                        elif backup is not None:
                            rf = orig_open(path, "wb")
                            try:
                                rf.write(backup)
                            finally:
                                rf.close()
                    raise OSError("close")

            return _W()
        return _open

    gl.log_reset()
    close_path = os.path.join(close_dir, "c.trk")
    make_gpx(close_path, 2)
    gl.log.update(file=close_path, created=True, points=2, batch=[extra_pt])
    builtins.open = _close_wrap(False)
    try:
        gl.flush()
        gl.flush()
    finally:
        builtins.open = orig_open
    check("ROT-11: close() with data on flash does not rewrite the batch",
          _gpx_pts(close_path) == 3 and gl.log["batch"] == [],
          "n_pts=%s batch=%s" % (_gpx_pts(close_path), gl.log["batch"]))

    # WF-5: LittleFS rollback on append — keep the batch, no lost points
    make_gpx(close_path, 2)
    gl.log.update(file=close_path, created=True, points=2, batch=[extra_pt],
                  err=0, new_seg=False, good_size=os.path.getsize(close_path))
    builtins.open = _close_wrap(True)
    try:
        gl.flush()
    finally:
        builtins.open = orig_open
    check("WF-5: append close rollback keeps the batch",
          extra_pt in gl.log["batch"] and _gpx_pts(close_path) == 2,
          "batch=%s n_pts=%s created=%s" % (
              gl.log["batch"], _gpx_pts(close_path), gl.log.get("created")))
    gl.flush()
    check("WF-5: later flush writes the kept points",
          _gpx_pts(close_path) == 3 and gl.log["batch"] == [],
          "n_pts=%s" % _gpx_pts(close_path))

    # WF-6: create close rollback — file missing, created=False, batch kept
    new_path = os.path.join(close_dir, "new.trk")
    gl.log.update(file=new_path, created=False, points=0, batch=[extra_pt],
                  err=0, new_seg=False)
    builtins.open = _close_wrap(True)
    try:
        gl.flush()
    finally:
        builtins.open = orig_open
    check("WF-6: create close rollback does not claim the file exists",
          gl.log["created"] is False and extra_pt in gl.log["batch"]
          and not os.path.exists(new_path),
          "created=%s exists=%s batch=%s" % (
              gl.log.get("created"), os.path.exists(new_path), gl.log["batch"]))
    gl.flush()
    check("WF-6: later flush creates the file with the kept points",
          gl.log["created"] is True and os.path.exists(new_path)
          and _gpx_pts(new_path) == 1 and gl.log["batch"] == [],
          "created=%s n_pts=%s" % (gl.log.get("created"), _gpx_pts(new_path)))

    # M3-3: MemoryError mid-write commits a partial file; retry must
    # overwrite from the last good footer (append) or rewrite (create).
    def _partial_open(fail_on, cut):
        writes = [0]

        def _open(path, *a, **k):
            inner = orig_open(path, *a, **k)

            class _W(object):
                def seek(self, *x, **y):
                    return inner.seek(*x, **y)

                def tell(self):
                    return inner.tell()

                def read(self, *x, **y):
                    return inner.read(*x, **y)

                def write(self, data):
                    writes[0] += 1
                    if writes[0] == fail_on:
                        inner.write(data[:cut])
                        raise MemoryError("heap")
                    return inner.write(data)

                def close(self):
                    inner.close()

            return _W()

        return _open

    part_path = os.path.join(close_dir, "part.trk")
    make_gpx(part_path, 2)
    good = os.path.getsize(part_path)
    gl.log.update(file=part_path, created=True, points=2, batch=[extra_pt],
                  err=0, new_seg=False, good_size=good)
    builtins.open = _partial_open(2, 3)  # point ok, 3 bytes of FOOTER
    try:
        gl.flush()
    finally:
        builtins.open = orig_open
    check("WF-7: append MemoryError keeps the batch and created",
          extra_pt in gl.log["batch"] and gl.log["created"] is True
          and gl.log.get("good_size") == good,
          "batch=%s created=%s good=%s" % (
              gl.log["batch"], gl.log.get("created"), gl.log.get("good_size")))
    gl.flush()
    with orig_open(part_path, "rb") as f:
        part_body = f.read()
    merged = extra_pt.encode()[:20] + extra_pt.encode()
    check("WF-7: append retry from good_size does not merge lines",
          _gpx_pts(part_path) == 3 and gl.log["batch"] == []
          and extra_pt.encode() in part_body and merged not in part_body
          and part_body.endswith(gl.FOOTER.encode()),
          "n_pts=%s tail=%r" % (_gpx_pts(part_path), part_body[-80:]))

    create_part = os.path.join(close_dir, "create_part.trk")
    gl.log.update(file=create_part, created=False, points=0, batch=[extra_pt],
                  err=0, new_seg=False, good_size=None)
    builtins.open = _partial_open(2, 10)  # header ok, 10 bytes of point
    try:
        gl.flush()
    finally:
        builtins.open = orig_open
    check("WF-8: create MemoryError leaves created=False",
          gl.log["created"] is False and extra_pt in gl.log["batch"]
          and gl.log.get("good_size") is None,
          "created=%s batch=%s good=%s" % (
              gl.log.get("created"), gl.log["batch"], gl.log.get("good_size")))
    gl.flush()
    with orig_open(create_part, "rb") as f:
        create_body = f.read()
    check("WF-8: create retry rewrites from scratch with a valid header",
          gl.log["created"] is True and gl.log["batch"] == []
          and _gpx_pts(create_part) == 1
          and create_body.startswith(gl.HEADER.encode())
          and create_body.endswith(gl.FOOTER.encode()),
          "created=%s n_pts=%s head=%r" % (
              gl.log.get("created"), _gpx_pts(create_part), create_body[:80]))

    # M7: transient stat/open EIO must not flip created=False (wb would wipe)
    eio_path = os.path.join(close_dir, "eio.trk")
    make_gpx(eio_path, 112)
    eio_pts = _gpx_pts(eio_path)
    gl.log.update(file=eio_path, created=True, points=112, batch=[extra_pt],
                  err=0, new_seg=False, good_size=os.path.getsize(eio_path))
    orig_stat = os.stat

    def _eio_open(path, *a, **k):
        raise OSError(5, "EIO")

    def _eio_stat(path):
        if path == eio_path:
            raise OSError(5, "EIO")
        return orig_stat(path)

    os.stat = _eio_stat
    builtins.open = _eio_open
    try:
        gl.flush()
    finally:
        builtins.open = orig_open
        os.stat = orig_stat
    check("WF-9: transient EIO keeps created=True and the trip on flash",
          gl.log["created"] is True and extra_pt in gl.log["batch"]
          and _gpx_pts(eio_path) == eio_pts,
          "created=%s n_pts=%s batch=%s" % (
              gl.log.get("created"), _gpx_pts(eio_path), gl.log["batch"]))
    gl.flush()
    check("WF-9: later flush appends without wiping the trip",
          _gpx_pts(eio_path) == eio_pts + 1 and gl.log["batch"] == [],
          "n_pts=%s" % _gpx_pts(eio_path))

    gone_path = os.path.join(close_dir, "gone.trk")
    make_gpx(gone_path, 2)
    gl.log.update(file=gone_path, created=True, points=2, batch=[extra_pt],
                  err=0, new_seg=False, good_size=os.path.getsize(gone_path))
    os.remove(gone_path)
    gl.flush()
    check("WF-9: ENOENT on append sets created=False",
          gl.log["created"] is False and extra_pt in gl.log["batch"],
          "created=%s batch=%s" % (gl.log.get("created"), gl.log["batch"]))
    gl.flush()
    check("WF-9: ENOENT retry recreates the file",
          gl.log["created"] is True and _gpx_pts(gone_path) == 1,
          "created=%s n_pts=%s" % (gl.log.get("created"), _gpx_pts(gone_path)))

    trunc_path = os.path.join(close_dir, "trunc.trk")
    make_gpx(trunc_path, 2)
    with orig_open(trunc_path, "rb") as f:
        trunc_raw = f.read()
    with orig_open(trunc_path, "wb") as f:
        f.write(trunc_raw[:-3])
    gl.log.update(file=trunc_path, created=True, points=2, batch=[extra_pt],
                  err=0, new_seg=False, good_size=len(trunc_raw))
    gl.flush()
    with orig_open(trunc_path, "rb") as f:
        trunc_body = f.read()
    check("WF-10: truncated footer is repaired before append",
          _gpx_pts(trunc_path) == 3 and gl.log["batch"] == []
          and trunc_body.endswith(gl.FOOTER.encode())
          and extra_pt.encode() in trunc_body,
          "n_pts=%s tail=%r" % (_gpx_pts(trunc_path), trunc_body[-80:]))

    # new trip during a write error starts a new track segment
    gl.log.update(file=close_path, created=True, points=2, batch=[extra_pt],
                  fix=False, lost_at=_clock[0] - gl.NEW_FILE_GAP * 1000 - 1,
                  last_flush=_clock[0], prev=(52.5, 13.4), new_seg=False,
                  online=False)
    gl.st.update(valid=True, date="2026-10-05", time="19:00:00",
                 lat=52.6, lon=13.5, alt=10.0, pos_at=_clock[0],
                 hdop=1.0, hdop_at=_clock[0], status="A", quality=1, fixtype=3)
    gl.log_tick()
    check("APRÓ-2: new trip while flush failed sets new_seg",
          gl.log.get("new_seg") is True and gl.log.get("file") == close_path
          and any("52.600000" in p for p in gl.log["batch"]),
          "new_seg=%s prev=%s file=%s batch=%s" % (
              gl.log.get("new_seg"), gl.log.get("prev"), gl.log.get("file"),
              gl.log.get("batch")))

    # --- empty / EOF .bad must be deleted, not spun (~50 read_batch/s) -----
    online(tmpdir)
    empty_bad = os.path.join(tmpdir, "empty.gpx.bad")
    with open(empty_bad, "wb"):
        pass
    reads = [0]
    orig_batch = gl.read_batch

    def _count_batch(*a, **k):
        reads[0] += 1
        return orig_batch(*a, **k)

    gl.read_batch = _count_batch
    gl.up["allow_bad"] = True
    gl.up["next_at"] = _clock[0]
    gl.upload_tick()
    check("BUSY: empty .bad is deleted",
          not os.path.exists(empty_bad),
          "exists=%s" % os.path.exists(empty_bad))
    n1 = reads[0]
    _clock[0] += 20
    gl.upload_tick()
    check("BUSY: empty .bad does not spin (next_at in the future)",
          reads[0] == n1,
          "reads %s -> %s next_at=%s now=%s" % (
              n1, reads[0], gl.up["next_at"], _clock[0]))

    eof_bad = os.path.join(tmpdir, "tail.gpx.bad")
    with open(eof_bad, "wb") as f:
        f.write(b'<trkpt lat="52.5" lon="13.4"><time>2026-10-05T00:00:00Z</time></trkpt>\n')
    gl.write_offset(eof_bad, os.path.getsize(eof_bad))
    gl.up["allow_bad"] = True
    gl.up["next_at"] = _clock[0]
    reads[0] = 0
    gl.upload_tick()
    check("BUSY: .bad with .pos at EOF is deleted (power-loss commit)",
          not os.path.exists(eof_bad) and not os.path.exists(eof_bad + ".pos"),
          "bad=%s pos=%s" % (os.path.exists(eof_bad),
                             os.path.exists(eof_bad + ".pos")))
    n2 = reads[0]
    _clock[0] += 20
    gl.upload_tick()
    check("BUSY: EOF .bad does not spin",
          reads[0] == n2,
          "reads %s -> %s" % (n2, reads[0]))
    gl.read_batch = orig_batch

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

# --- Review fixes: home gate, .env comments, ticks wrap, NMEA validity ------
gl = _reload_logger()
rv_dir = tempfile.mkdtemp(prefix="gpxlog-rv-")
try:
    gl.LOG_DIR = rv_dir
    gl.reset()
    gl.log_reset()
    gl.wifi_reset()
    gl.up_reset()
    rv_sec = [0]

    def rv_fix(lat, speed):
        s = rv_sec[0]
        rv_sec[0] += 1
        gl.st.update(valid=True, date="2026-10-17",
                     time="%02d:%02d:%02d" % (12 + s // 3600, (s // 60) % 60,
                                              s % 60),
                     lat=lat, lon=19.0, alt=100.0, pos_at=_clock[0],
                     hdop=1.0, hdop_at=_clock[0], status="A", quality=1,
                     fixtype=3, speed=speed)

    # REVIEW-2: trailing comments on flag / IP / name keys
    env_p = os.path.join(rv_dir, "env")
    with open(env_p, "w") as f:
        f.write("SSID=home\nWPA2=pa#ss word\n"
                "IP=192.168.1.60  # car\nGW=192.168.1.1\n"
                "MASK=255.255.255.0   # optional\n"
                "DNS=192.168.1.111    # optional, defaults to GW\n"
                "SSID2=Phone#1\nWPA2_2=x\nHOME2=1  # treat as home\n"
                "IP2=10.0.0.5\nGW2=10.0.0.1\nDNS2=10.0.0.999\n"
                "DEVICE_ID=car-1 # id\n")
    rv_nets = gl._load_nets(gl.load_env(env_p))
    check("REVIEW-2: trailing comment does not drop DNS to the gateway",
          rv_nets[0]["ip"] == ("192.168.1.60", "255.255.255.0",
                               "192.168.1.1", "192.168.1.111"),
          repr(rv_nets[0]["ip"]))
    check("REVIEW-2: password and SSID with '#' are kept verbatim",
          rv_nets[0]["pw"] == "pa#ss word" and rv_nets[1]["ssid"] == "Phone#1")
    check("REVIEW-2: HOME2 flag with a trailing comment is honoured",
          rv_nets[1]["home"] is True)
    check("REVIEW-2: invalid DNS2 falls back to GW2",
          rv_nets[1]["ip"] == ("10.0.0.5", "255.255.255.0", "10.0.0.1",
                               "10.0.0.1"), repr(rv_nets[1]["ip"]))
    reset_wlan()
    gl.ENV_FILE = env_p
    gl.wifi_reset()
    gl.wifi_start()
    check("REVIEW-2: DEVICE_ID with a trailing comment is accepted",
          gl.DEVICE_ID == "car-1" and gl.HOSTNAME == "car-1",
          "%s %s" % (gl.DEVICE_ID, gl.HOSTNAME))
    with open(os.path.join(ROOT, "README.md")) as f:
        readme = f.read()
    blk = readme[readme.index("IP=192.168.1.60"):]
    blk = blk[:blk.index("```")]
    with open(env_p, "w") as f:
        f.write("SSID=home\n" + blk)
    rv_nets = gl._load_nets(gl.load_env(env_p))
    check("REVIEW-2: README static-IP example parses as documented",
          rv_nets[0]["ip"] == ("192.168.1.60", "255.255.255.0",
                               "192.168.1.1", "192.168.1.111"),
          repr(rv_nets[0]["ip"]))
    gl.wifi_reset()

    # REVIEW-3: ticks_ms wraps at 2**30 and ticks_diff is signed
    TP = 1 << 30
    orig_ticks = (time.ticks_ms, time.ticks_diff, time.ticks_add)
    time.ticks_ms = lambda: _clock[0] & (TP - 1)
    time.ticks_diff = lambda a, b: ((a - b + TP // 2) & (TP - 1)) - TP // 2
    time.ticks_add = lambda a, b: (a + b) & (TP - 1)
    saved_clock = _clock[0]
    try:
        _clock[0] = (1 << 29) + 1000        # ~6.2 days after ticks 0
        now = _clock[0]
        check("REVIEW-3: a wrapped age counts as very old",
              gl._age(now, 0) == gl.TICKS_OLD and gl._age(now, now - 5) == 5)
        check("REVIEW-3: a wrapped deadline is due, a near one is pending",
              not gl._waiting(now, 0, gl.UP_WAIT_MAX)
              and gl._waiting(now, now + 1000, gl.UP_WAIT_MAX))
        gl.reset()
        gl.log_reset()
        gl.wifi_reset()
        gl.wifi["failed"]["home"] = 0
        check("REVIEW-3: a failure 6 days ago is not 'recently failed'",
              gl._recently_failed("home", now) is False)
        gl.st.update(valid=True, date="2026-10-17", time="12:00:00",
                     lat=47.5, lon=19.0, pos_at=0)
        check("REVIEW-3: a 6-day-old position is not a fix",
              not gl.has_fix(now))
        for start in (None, 0):
            gl.log_reset()
            gl.log.update(full=True, full_check=start)
            gl.log_tick()
            check("REVIEW-3: full flash is re-polled (full_check=%r)" % start,
                  gl.log["full"] is False and gl.log["full_check"] == now)
        gl.log_reset()
        gl.log.update(last_sample=0, last_flush=0)
        rv_fix(47.5, 30.0)
        gl.log_tick()
        # the sample opens a stay window, so the flush holds it in RAM
        check("REVIEW-3: wrapped last_sample/last_flush still sample + flush",
              gl.log["points"] + len(gl.log["batch"]) == 1
              and gl.log["last_flush"] == now,
              "points=%s batch=%s flush=%s" % (
                  gl.log["points"], len(gl.log["batch"]), gl.log["last_flush"]))
        gl.flush()
        done = gl.log["file"]
        gl.log.update(file=None)
        for n in os.listdir(rv_dir):         # leave only this track
            if os.path.join(rv_dir, n) != done:
                os.remove(os.path.join(rv_dir, n))
        gl.up.update(url="https://d.example", key="k", state="idle",
                     next_at=0, hold={done: 0})
        gl.wifi.update(state="connected", home=True)
        _post_code[0] = 201
        _posts[:] = []
        gl.upload_tick()
        check("REVIEW-3: wrapped next_at / hold do not block uploads",
              len(_posts) == 1 and not os.path.exists(done),
              "posts=%s state=%s" % (len(_posts), gl.up["state"]))
    finally:
        time.ticks_ms, time.ticks_diff, time.ticks_add = orig_ticks
        _clock[0] = saved_clock

    # REVIEW-4 / AVG: a second becomes the fix when the next one starts;
    # its validity is that of its own sentences, its position their mean
    def nmea(body):
        c = 0
        for ch in body.encode():
            c ^= ch
        return ("$%s*%02X" % (body, c)).encode()

    def epoch(hhmmss, lat="4730.000", lon="01900.000", status="A", q=1,
              hdop="1.0", alt="100.0", date="171026", kn="2.0"):
        gl.handle(nmea("GPRMC,%s,%s,%s,N,%s,E,%s,90.0,%s,,,A"
                       % (hhmmss, status, lat, lon, kn, date)))
        gl.handle(nmea("GPGGA,%s,%s,N,%s,E,%d,08,%s,%s,M,40.0,M,,"
                       % (hhmmss, lat, lon, q, hdop, alt)))

    gl.reset()
    epoch("120000.00")
    check("AVG: a second is not a fix while it is still being collected",
          not gl.has_fix(_clock[0]) and gl.st["seq"] == 0)
    epoch("120001.00")
    check("REVIEW-4: valid RMC + GGA is a fix (once the next second starts)",
          bool(gl.has_fix(_clock[0])) and gl.st["time"] == "12:00:00"
          and gl.st["date"] == "2026-10-17" and gl.st["seq"] == 1)
    _clock[0] += 1000
    gl.handle(nmea("GPRMC,120002.00,V,,,,,,,171026,,,N"))  # 12:00:01 done
    fix_at = gl.st["pos_at"]
    gl.handle(nmea("GPGGA,120002.00,,,,,0,00,99.9,,M,,M,,"))
    gl.handle(nmea("GPRMC,120003.00,V,,,,,,,171026,,,N"))
    check("REVIEW-4: a void second is no fix and does not refresh pos_at",
          gl.st["valid"] is False and gl.st["pos_at"] == fix_at
          and gl.st["time"] == "12:00:02")
    _clock[0] += 1000
    gl.handle(nmea("GPRMC,120004.00,A,,,,,,,171026,,,A"))
    gl.handle(nmea("GPGGA,120004.00,,,,,1,08,1.0,,M,,M,,"))
    gl.handle(nmea("GPRMC,120005.00,V,,,,,,,171026,,,N"))
    check("REVIEW-4: a valid second without a position is no fix",
          gl.st["valid"] is False and gl.st["pos_at"] == fix_at)

    # AVG: five fixes of one second publish their mean, once
    gl.reset()
    lats = ("4730.0000", "4730.0010", "4730.0020", "4730.0030", "4730.0040")
    for k, la in enumerate(lats):
        epoch("120010.%02d" % (k * 20), lat=la, hdop="%.1f" % (1.0 + k * 0.1))
    epoch("120011.00", lat="4731.0000")
    want = 47.5 + 0.0020 / 60
    check("AVG: five fixes of a second publish their mean position",
          gl.st["epochs"] == 5 and abs(gl.st["lat"] - want) < 1e-7
          and gl.st["seq"] == 1 and abs(gl.st["hdop"] - 1.2) < 1e-6,
          "lat=%r epochs=%s hdop=%s" % (gl.st["lat"], gl.st["epochs"],
                                        gl.st["hdop"]))
    # AVG: moving faster than AVG_MAX_SPEED the point is the second's first
    # fix (the mean would lie ~0.4 s along the track, stamped the second)
    gl.reset()
    for k, la in enumerate(lats):
        epoch("120010.%02d" % (k * 20), lat=la, kn="20.0" if k == 2 else "2.0",
              hdop="%.1f" % (1.0 + k * 0.1))
    epoch("120011.00", lat="4731.0000")
    check("AVG: above AVG_MAX_SPEED the point is the second's first fix",
          gl.st["epochs"] == 5 and abs(gl.st["lat"] - 47.5) < 1e-7
          and abs(gl.st["hdop"] - 1.2) < 1e-6,
          "lat=%r hdop=%s" % (gl.st["lat"], gl.st["hdop"]))
    gl.reset()
    for k in range(5):
        epoch("120020.%02d" % (k * 20), q=0 if k < 3 else 1,
              lat="4730.%04d" % (k * 10))
    epoch("120021.00")
    check("AVG: only the second's valid fixes are averaged",
          gl.st["epochs"] == 2
          and abs(gl.st["lat"] - (47.5 + 0.0035 / 60)) < 1e-7,
          "epochs=%s lat=%r" % (gl.st["epochs"], gl.st["lat"]))
    gl.reset()
    gl.handle(nmea("GPRMC,120030.00,A,4730.000,N,01900.000,E,1.0,0.0,"
                   "171026,,,A"))
    gl.handle(nmea("GPRMC,120031.00,A,4730.000,N,01900.000,E,1.0,0.0,"
                   "171026,,,A"))
    check("AVG: RMC positions are used when a second has no GGA",
          bool(gl.has_fix(_clock[0])) and gl.st["epochs"] == 1)

    # AVG: the logger records one point per averaged second, its mean
    avg_dir = tempfile.mkdtemp(prefix="gpxlog-avg-")
    saved_dir = gl.LOG_DIR
    try:
        gl.LOG_DIR = avg_dir
        gl.reset()
        gl.log_reset()
        gl.wifi_reset()
        for sec in range(30):
            for k in range(5):                    # 5 Hz, 1.4 m/s north
                north = (sec * 5 + k) * 0.28
                epoch("1201%02d.%02d" % (sec, k * 20),
                      lat="4730.%04d" % int(round(north / 1.852 * 10000 / 1000)))
                _clock[0] += 200
                gl.log_tick()
        lines = list(gl.log["batch"])
        secs = [l.strip().split(",")[4] for l in lines]
        check("AVG: 5 Hz in, at most one point a second out, in time order",
              len(lines) >= 5 and len(set(secs)) == len(secs)
              and secs == sorted(secs), "points=%d" % len(lines))
    finally:
        gl.LOG_DIR = saved_dir
        shutil.rmtree(avg_dir, ignore_errors=True)

    # AVG: seconds published together (after a blocking POST) all count
    q_dir = tempfile.mkdtemp(prefix="gpxlog-q-")
    saved_dir = gl.LOG_DIR
    try:
        gl.LOG_DIR = q_dir
        gl.reset()
        gl.log_reset()
        gl.wifi_reset()
        epoch("120200.00", lat="4730.0000")
        epoch("120201.00", lat="4730.0100")
        gl.log_tick()                                # first second
        n0 = len(gl.log["batch"])
        for k, la in ((2, "4730.0200"), (3, "4730.0300"), (4, "4730.0400")):
            epoch("12020%d.00" % k, lat=la)
            _clock[0] += 1000                        # a 3 s POST: no ticks
        epoch("120205.00", lat="4730.0500")
        gl.log_tick()
        secs = [l.strip().split(",")[4][9:15] for l in gl.log["batch"]]
        check("AVG: seconds published during a blocking call are all points",
              len(gl.log["batch"]) - n0 == 4
              and secs == ["120200", "120201", "120202", "120203", "120204"]
              and gl.st["time"] == "12:02:04", "secs=%s" % secs)
    finally:
        gl.LOG_DIR = saved_dir
        shutil.rmtree(q_dir, ignore_errors=True)

    # AVG: valid seconds published just before a fix loss still count
    q_dir = tempfile.mkdtemp(prefix="gpxlog-q2-")
    saved_dir = gl.LOG_DIR
    try:
        gl.LOG_DIR = q_dir
        gl.reset()
        gl.log_reset()
        gl.wifi_reset()
        epoch("120300.00", lat="4730.0000")
        epoch("120301.00", lat="4730.0100")
        gl.log_tick()
        n0 = len(gl.log["batch"])
        for k, la in ((2, "4730.0200"), (3, "4730.0300")):
            epoch("12030%d.00" % k, lat=la)
            _clock[0] += 1000                        # a POST: no ticks
        epoch("120304.00", q=0, status="V")          # the fix is lost
        epoch("120305.00", q=0, status="V")
        gl.log_tick()
        check("AVG: seconds before a fix loss in the same burst are points",
              len(gl.log["batch"]) - n0 == 3 and not gl.log["fix"],
              "new=%d" % (len(gl.log["batch"]) - n0))
    finally:
        gl.LOG_DIR = saved_dir
        shutil.rmtree(q_dir, ignore_errors=True)

    # UBX: the setup frames (checked against u-blox reference frames)
    _uart_tx[:] = []
    gl.gps_setup()
    frames = [f.hex() for f in _uart_tx]
    check("UBX: CFG-MSG turns GSV off (b562 0601 0300 f00300 fd15)",
          "b56206010300f00300fd15" in frames, repr(frames[:2]))
    check("UBX: CFG-RATE asks for 200 ms (b562 0608 ... de6a)",
          frames[-1] == "b56206080600c800010001" + "00de6a", frames[-1])
    check("UBX: GSA once a second, RMC and GGA every fix",
          gl._ubx(6, 1, bytes((0xF0, 2, gl.GPS_HZ))).hex() in frames
          and gl._ubx(6, 1, bytes((0xF0, 0, 1))).hex() in frames
          and gl._ubx(6, 1, bytes((0xF0, 4, 1))).hex() in frames)
    nav5 = gl._nav5()
    pl = nav5[6:-2]
    check("UBX: CFG-NAV5 (36 B) sets the model, 3D only, 10 deg, static hold",
          nav5[2:4] == b"\x06\x24" and len(pl) == 36
          and pl[1] == 0 and pl[2] == gl.GPS_DYN_MODEL
          and pl[3] == gl.GPS_FIX_MODE and pl[12] == gl.GPS_MIN_ELEV
          and pl[22] == gl.GPS_STATIC_HOLD and nav5.hex() in frames
          and frames.index(nav5.hex()) == 1, nav5.hex())
    # u-blox CFG-NAV5 mask: 0x01 dyn, 0x02 minEl, 0x04 fixMode, 0x08 drLim,
    # 0x10 posMask, 0x20 timeMask, 0x40 staticHoldMask
    check("UBX: NAV5 mask applies dyn, minEl, fixMode and static hold only",
          pl[0] == 0x01 | 0x02 | 0x04 | 0x40 and not pl[0] & 0x10
          and not pl[0] & 0x20, "mask=0x%02x" % pl[0])
    check("UBX: the default dynamic model is portable (car and foot alike)",
          gl.GPS_DYN_MODEL == 0)
    # NAV5 refused (NAK) -> the 5 Hz setup goes on; silent -> stays at 1 Hz
    _uart_tx[:] = []
    _uart_nak.add((0x06, 0x24))
    try:
        gl.gps_setup()
    finally:
        _uart_nak.discard((0x06, 0x24))
    check("UBX: a NAV5 NAK does not stop the 5 Hz setup",
          gl.st["setup_ok"] is True and gl.st["nav5_ok"] is False
          and _uart_tx[-1].hex() == gl._rate(1000 // gl.GPS_HZ).hex(),
          "ok=%s nav5=%s" % (gl.st["setup_ok"], gl.st["nav5_ok"]))
    _uart_tx[:] = []
    _uart_silent.add((0x06, 0x24))
    t0 = _clock[0]
    try:
        gl.gps_setup()
    finally:
        _uart_silent.discard((0x06, 0x24))
    check("UBX: a receiver silent at NAV5 stays at 1 Hz, ~4 s block",
          gl.st["setup_ok"] is False
          and gl._rate(1000 // gl.GPS_HZ).hex()
          not in [f.hex() for f in _uart_tx]
          and _clock[0] - t0 <= 2 * gl.GPS_ACK_TRIES * gl.GPS_ACK_WAIT,
          "ms=%d" % (_clock[0] - t0))
    _uart_tx[:] = []
    gl.gps_setup()
    check("UBX: setup acknowledged again afterwards (NAV5 accepted)",
          gl.st["setup_ok"] is True and gl.st["nav5_ok"] is True)
    # GPS_MODEL from .env, read before the first setup
    saved_model = gl.GPS_DYN_MODEL
    try:
        gl._gps_model_env({"GPS_MODEL": "3  # on foot"})
        m3 = gl.GPS_DYN_MODEL
        gl._gps_model_env({"GPS_MODEL": "1"})
        m_bad = gl.GPS_DYN_MODEL
        gl._gps_model_env({"GPS_MODEL": "car"})
        m_bad2 = gl.GPS_DYN_MODEL
        gl.GPS_DYN_MODEL = 4
        gl_nav = gl._nav5()[6 + 2]
    finally:
        gl.GPS_DYN_MODEL = saved_model
    import inspect
    run_src = inspect.getsource(gl.run)
    check("UBX: .env GPS_MODEL sets the model (invalid kept), before the setup",
          m3 == 3 and m_bad == 3 and m_bad2 == 3 and gl_nav == 4
          and run_src.index("_gps_model_env") < run_src.index("gps_setup()"),
          "%s %s %s %s" % (m3, m_bad, m_bad2, gl_nav))
    # reset loop: banner starts over and over, no NMEA -> warned and counted
    out = []
    real_print = gl.print if hasattr(gl, "print") else None
    gl.print = lambda *a, **k: out.append(" ".join(str(x) for x in a))
    try:
        gl.reset()
        for _ in range(4):
            _uart_rx.extend(b"$GPTXT,01,01,02,u\xff\xff\xff" * 3)
            gl.read_uart()
            _clock[0] += gl.GPS_LOOP_WINDOW // 4 + 10
            gl.gps_check(_clock[0])
        looped = gl.st["loops"]
        warned = any("gps reset loop" in o for o in out)
        gl.reset()
        _uart_rx.extend(nmea("GPTXT,01,01,02,u-blox ag - www.u-blox.com")
                        + b"\r\n")
        for k in range(12):
            _uart_rx.extend(nmea("GPRMC,1205%02d.00,V,,,,,,,171026,,,N" % k)
                            + b"\r\n")
            gl.read_uart()
            _clock[0] += 1000
            gl.gps_check(_clock[0])
        normal = gl.st["loops"]
        # restarts with NMEA in between: GPS_FLAP_RESTARTS in the window
        banner = (nmea("GPTXT,01,01,02,u-blox ag - www.u-blox.com")
                  + b"\r\n")
        def restarts(n, gap):
            for k in range(n):
                _uart_rx.extend(banner)
                for j in range(3):
                    _uart_rx.extend(
                        nmea("GPRMC,1206%02d.00,V,,,,,,,171026,,,N" % j)
                        + b"\r\n")
                gl.read_uart()
                _clock[0] += gap
                gl.gps_check(_clock[0])
        gl.reset()
        out[:] = []
        restarts(gl.GPS_FLAP_RESTARTS, 10000)
        flap = gl.st["loops"]
        flap_warned = any("gps reset loop" in o and "restarts" in o
                          for o in out)
        flap_ok = gl.st["ok"]
        restarts(gl.GPS_FLAP_RESTARTS - 1, 10000)
        flap_again = gl.st["loops"]
        restarts(1, 10000)
        flap_twice = gl.st["loops"]
        gl.reset()
        restarts(gl.GPS_FLAP_RESTARTS * 2,
                 gl.GPS_FLAP_WINDOW // (gl.GPS_FLAP_RESTARTS - 1) + 1000)
        flap_slow = gl.st["loops"]
    finally:
        if real_print is None:
            del gl.print
        else:
            gl.print = real_print
    check("LOOP: banner starts and no NMEA for a window -> 'gps reset loop?'",
          looped == 1 and warned, "loops=%d out=%s" % (looped, out[-2:]))
    check("LOOP: one start-up banner followed by NMEA is not a loop",
          normal == 0, "loops=%d" % normal)
    check("LOOP: GPS_FLAP_RESTARTS restarts with NMEA in between -> warned",
          flap == 1 and flap_warned and flap_ok > 0,
          "loops=%d ok=%d out=%s" % (flap, flap_ok, out[-2:]))
    check("LOOP: after a warning the next restarts are counted afresh",
          flap_again == 1 and flap_twice == 2,
          "%d %d" % (flap_again, flap_twice))
    check("LOOP: restarts spread wider than GPS_FLAP_WINDOW are no loop",
          flap_slow == 0, "loops=%d" % flap_slow)
    # ACK: 1 Hz first, every frame until acknowledged, stop if nobody answers
    _uart_tx[:] = []
    gl.gps_setup()
    rate1000 = gl._rate(1000).hex()
    check("ACK: the setup starts at 1 Hz and ends at GPS_HZ, all acknowledged",
          _uart_tx[0].hex() == rate1000
          and _uart_tx[-1].hex() == gl._rate(1000 // gl.GPS_HZ).hex()
          and len(_uart_tx) == len(gl.GPS_MSGS) + 3
          and gl.st["setup_ok"] is True)
    _uart_tx[:] = []
    _uart_drop[0] = 2                               # two frames lost
    gl.gps_setup()
    check("ACK: a dropped frame is sent again until acknowledged",
          len(_uart_tx) == len(gl.GPS_MSGS) + 5
          and _uart_tx[0] == _uart_tx[1] == _uart_tx[2]
          and gl.st["setup_ok"] is True, "frames=%d" % len(_uart_tx))
    _uart_tx[:] = []
    _uart_ack[0] = False
    t0 = _clock[0]
    try:
        gl.gps_setup()
    finally:
        _uart_ack[0] = True
    rates = (rate1000, gl._rate(1000 // gl.GPS_HZ).hex())
    check("ACK: no answer -> GPS_ACK_TRIES tries, then the message frames once",
          [f.hex() for f in _uart_tx[:gl.GPS_ACK_TRIES]]
          == [rate1000] * gl.GPS_ACK_TRIES
          and len(_uart_tx) == gl.GPS_ACK_TRIES + len(gl.GPS_MSGS)
          and not any(f.hex() in rates for f in _uart_tx[gl.GPS_ACK_TRIES:])
          and gl.st["setup_ok"] is False
          and _clock[0] - t0 <= gl.GPS_ACK_TRIES * gl.GPS_ACK_WAIT + 400,
          "frames=%d ms=%d" % (len(_uart_tx), _clock[0] - t0))
    check("ACK: GST is asked once a second",
          gl._ubx(6, 1, bytes((0xF0, 7, gl.GPS_HZ))).hex()
          in [f.hex() for f in (_uart_tx or [])] or (0x07, gl.GPS_HZ)
          in gl.GPS_MSGS)

    # TXT: a u-blox start-up banner counts a receiver restart
    gl.reset()
    gl.handle(nmea("GPTXT,01,01,02,u-blox ag - www.u-blox.com"))
    gl.handle(nmea("GPTXT,01,01,02,ANTSTATUS=OK"))
    check("TXT: the u-blox banner counts one receiver restart",
          gl.st["gps_boots"] == 1)
    import inspect
    run_src = inspect.getsource(gl.run)
    check("HOLD: Ctrl-C / crash flushes keep the held drift out",
          run_src.count("flush(_still_hold())") == 3
          and "flush()" not in run_src)

    # GST: the receiver's own error estimate keeps a converging cold start
    # out (good HDOP, hundreds of metres off), a settled fix is used
    def gst_second(hhmmss, std):
        epoch(hhmmss)
        gl.handle(nmea("GPGST,%s,20,,,,%.1f,%.1f,%.1f"
                       % (hhmmss, std, std * 0.8, std * 2)))

    gl.reset()
    gst_second("120100.00", 150.0)
    epoch("120101.00")
    check("GST: an estimate of 150 m makes the second unusable",
          gl.st["std"] == 150.0 and not gl._usable(_clock[0])
          and gl._sample_line(_clock[0], None, None) is None)
    gl.reset()
    gst_second("120110.00", 4.0)
    epoch("120111.00")
    check("GST: an estimate of 4 m is usable",
          gl.st["std"] == 4.0 and gl._usable(_clock[0])
          and gl._sample_line(_clock[0], None, None) is not None)
    gl.reset()
    epoch("120120.00")
    epoch("120121.00")
    check("GST: without GST (another receiver) only HDOP decides",
          gl.st["std"] is None and gl._usable(_clock[0]))

    # UBX: a receiver still booting at setup stays at 1 Hz -> sent again,
    # less and less often; one that took it is left alone
    gsv_line = nmea("GPGSV,3,1,12,01,40,083,46,02,17,308,41,12,07,344,39,"
                    "14,22,228,45")
    gl.reset()
    _uart_tx[:] = []
    gl.gps_setup()
    n_frames = len(_uart_tx)
    gl.handle(gsv_line)                             # still in the UART buffer
    _clock[0] += gl.GPS_RETRY
    gl.gps_check(_clock[0])
    check("UBX: sentences queued before the setup do not count",
          len(_uart_tx) == n_frames and gl.st["extra"] == 0)
    gl.handle(gsv_line)
    gl.handle(nmea("GPVTG,,T,,M,0.0,N,0.0,K,A"))
    gl.gps_check(_clock[0])
    check("UBX: GSV / VTG after the setup -> it is sent again",
          len(_uart_tx) == 2 * n_frames and gl.st["extra"] == 0
          and gl.st["setup_wait"] == gl.GPS_RETRY)
    _clock[0] += 1500
    gl.handle(gsv_line)
    _clock[0] += gl.GPS_RETRY
    gl.gps_check(_clock[0])
    check("UBX: a receiver that acknowledges is set up again every GPS_RETRY",
          len(_uart_tx) == 3 * n_frames and gl.st["setup_wait"] == gl.GPS_RETRY)
    _clock[0] += 10 * gl.GPS_RETRY_MAX
    gl.gps_check(_clock[0])
    check("UBX: a receiver that took it is never sent it again",
          len(_uart_tx) == 3 * n_frames)
    # a receiver that never answers: less and less often
    _uart_ack[0] = False
    try:
        gl.reset()
        gl.gps_setup()
        sends = []
        for _ in range(40):
            _clock[0] += 1500
            gl.handle(gsv_line)
            n0 = len(_uart_tx)
            gl.gps_check(_clock[0])
            if len(_uart_tx) > n0:
                sends.append(_clock[0])
        gaps = [(b2 - a2) // 1000 for a2, b2 in zip(sends, sends[1:])]
    finally:
        _uart_ack[0] = True
    check("UBX: a silent receiver is retried with a doubling wait",
          len(gaps) >= 2 and gaps == sorted(gaps) and gaps[-1] > gaps[0],
          "gaps=%s" % gaps)
    # a restart banner sets it up again within GPS_RETRY
    gl.reset()
    gl.gps_setup()
    gl.st["setup_wait"] = gl.GPS_RETRY_MAX
    n0 = len(_uart_tx)
    _clock[0] += gl.GPS_RETRY
    gl.handle(nmea("GPTXT,01,01,02,u-blox ag - www.u-blox.com"))
    gl.gps_check(_clock[0])
    check("UBX: a restart banner sends the setup again at once",
          len(_uart_tx) > n0 and gl.st["gps_boots"] == 1)
    # a receiver that stops answering mid-setup blocks ~4 s, not ~36 s
    gl.reset()
    _uart_tx[:] = []
    real_write = UART.write
    calls = [0]

    def silent_after_one(self, data):
        """Acknowledge only the first frame (the 1 Hz one)."""
        calls[0] += 1
        if calls[0] == 1:
            return real_write(self, data)
        _uart_tx.append(bytes(data))
        return len(data)
    UART.write = silent_after_one
    t0 = _clock[0]
    try:
        gl.gps_setup()
    finally:
        UART.write = real_write
    check("UBX: a receiver silent after the first frame blocks ~4 s",
          gl.st["setup_ok"] is False
          and _clock[0] - t0 <= 2 * gl.GPS_ACK_TRIES * gl.GPS_ACK_WAIT + 500
          and len(_uart_tx) == 1 + gl.GPS_ACK_TRIES + len(gl.GPS_MSGS),
          "ms=%d frames=%d" % (_clock[0] - t0, len(_uart_tx)))
    check("UBX: ... and stays at 1 Hz: no GPS_HZ frame after an unanswered one",
          gl._rate(1000 // gl.GPS_HZ).hex()
          not in [f.hex() for f in _uart_tx])
    out = []
    real_print = gl.print if hasattr(gl, "print") else None
    gl.print = lambda *a, **k: out.append(" ".join(str(x) for x in a))
    try:
        gl.log_reset()
        gl.up["sent"] = 0
        gl._status()
    finally:
        if real_print is None:
            del gl.print
        else:
            gl.print = real_print
    check("STATUS: the console line works without a temperature sensor",
          out and out[0].startswith("status temp None hz"), repr(out))

    gl.reset()
    st_ok = gl.st["ok"]
    gl.handle(b"\xb5\x62\x05\x01\x02\x00\x06\x01\x0f\x38"
              + nmea("GPGSA,A,3,01,02,03,,,,,,,,,,2.0,1.0,1.7"))
    check("UBX: an ACK that runs into an NMEA line does not lose the line",
          gl.st["ok"] == st_ok + 1 and gl.st["fixtype"] == 3)

    # REVIEW3-2: a strong, visible candidate gets one immediate retry; the
    # fail / retry lines carry the raw status
    import contextlib
    import io

    def rv_start_wifi(env_text, aps, ok, fail_first=(), fail_always=()):
        reset_wlan()
        WLAN.scan_aps = aps
        WLAN.ok = set(ok)
        write_env(rv_dir, env_text)
        gl.wifi_reset()
        gl.up_reset()
        calls = []
        orig_connect = WLAN.connect

        def connect(self, ssid, pw, bssid=None):
            calls.append(ssid)
            if ssid in fail_always or (ssid in fail_first
                                       and calls.count(ssid) == 1):
                WLAN.ok.discard(ssid)
            elif ssid in ok:
                WLAN.ok.add(ssid)
            return orig_connect(self, ssid, pw, bssid)

        WLAN.connect = connect
        out = io.StringIO()
        try:
            with contextlib.redirect_stdout(out):
                gl.wifi_start()
                drain_wifi()
        finally:
            WLAN.connect = orig_connect
        return gl.wifi, calls, out.getvalue()

    rv_env = ("SSID=home\nWPA2=h\nSSID2=phone\nWPA2_2=p\n"
              "GEO_URL=https://d.example\nAPI_KEY=k\n")
    w, calls, out = rv_start_wifi(rv_env, [("home", -40), ("phone", -50)],
                                  ["home", "phone"], fail_first=("home",))
    check("REVIEW3-2: first home attempt fails, the retry connects home",
          w["state"] == "connected" and w["ssid"] == "home"
          and calls == ["home", "home"], "calls=%s state=%s ssid=%s" % (
              calls, w["state"], w["ssid"]))
    check("REVIEW3-2: the retry line carries the raw status",
          "wifi retry home status" in out and "wifi fail home" not in out,
          out)
    check("REVIEW3-2: a successful retry leaves the SSID not failed",
          "home" not in w["failed"])

    w, calls, out = rv_start_wifi(rv_env, [("home", -40), ("phone", -50)],
                                  ["home", "phone"], fail_always=("home",))
    check("REVIEW3-2: a retry that fails too moves on to the next network",
          w["ssid"] == "phone" and w["state"] == "connected"
          and calls == ["home", "home", "phone"] and "home" in w["failed"],
          "calls=%s" % calls)
    check("REVIEW3-2: one 'wifi fail' line with the status per demotion",
          out.count("wifi fail home status") == 1, out)

    w, calls, out = rv_start_wifi(rv_env, [("home", -80), ("phone", -85)],
                                  ["home", "phone"], fail_always=("home",))
    check("REVIEW3-2: a weak candidate (< ROAM_MIN_RSSI) is not retried",
          calls == ["home", "phone"] and w["ssid"] == "phone",
          "calls=%s" % calls)

    w, calls, out = rv_start_wifi(rv_env, [], ["home", "phone"],
                                  fail_always=("home",))
    check("REVIEW3-2: an SSID missing from the scan is not retried",
          calls == ["home", "phone"] and w["ssid"] == "phone",
          "calls=%s" % calls)
    gl.wifi_reset()

    # STAY: position-based stationary filter. Indoors the GPS speed is noise
    # (measured on the board lying still: p50 5.4 km/h, max 22 km/h), so
    # every fix below carries a random 0-22 km/h speed unless it is a drive.
    import random
    M_LAT = 1 / 111195.0                       # degrees per metre (lat)
    M_LON = M_LAT / 0.6755                     # at 47.5° N
    rnd = random.Random(1016)
    NOISE = "noise"

    def sv_reset(home=False):
        gl.reset()
        gl.log_reset()
        gl.wifi_reset()
        gl.up_reset()
        gl.wifi.update(state="connected" if home else "offline", home=home)
        for n in os.listdir(rv_dir):
            os.remove(os.path.join(rv_dir, n))

    def sv_fix(east, north, speed=NOISE, alt=100.0):
        """Fix east/north metres from 47.5, 19.0; one SAMPLE later."""
        if speed == NOISE:
            speed = rnd.uniform(0, 22)
        _clock[0] += gl.SAMPLE
        sec = rv_sec[0]
        rv_sec[0] += 1
        gl.st.update(valid=True, date="2026-10-18",
                     time="%02d:%02d:%02d" % (8 + sec // 3600,
                                              (sec // 60) % 60, sec % 60),
                     lat=47.5 + north * M_LAT, lon=19.0 + east * M_LON,
                     alt=alt, pos_at=_clock[0], hdop=1.0, hdop_at=_clock[0],
                     status="A", quality=1, fixtype=3, speed=speed)
        gl.log_tick()

    def sv_lines():
        out = []
        for n in sorted(os.listdir(rv_dir)):
            if n.endswith(".trk"):
                with open(os.path.join(rv_dir, n)) as f:
                    out.extend(l for l in f if l.count(",") == 4)
        return out + list(gl.log["batch"])

    def sv_m(line):
        f = line.split(",")
        return gl._dist(47.5, 19.0, float(f[0]), float(f[1]))

    def sv_drift(seconds, box=40.0, step=8.0, speed=NOISE):
        """Indoor drift: a random walk within +-box m (p90 ~ box)."""
        e = n = 0.0
        for _ in range(seconds):
            e = max(-box, min(box, e + rnd.uniform(-step, step)))
            n = max(-box, min(box, n + rnd.uniform(-step, step)))
            sv_fix(e, n, speed, alt=100 + rnd.uniform(-25, 25))

    def sv_active():
        return gl.log["still"]["active"]

    # away from home: 30 min of indoor drift with speed noise
    sv_reset()
    sv_drift(150)
    check("STAY: drift with 0-22 km/h speed noise becomes a stay in 2.5 min",
          sv_active() is True and gl.log["still"]["home"] is False)
    sv_drift(30 * 60 - 150)
    gl.flush()
    lines = sv_lines()
    beats = [l for l in lines if l.split(",")[2] == ""]
    # only the drift of the two windows that prove the stay can be on flash
    check("STAY: 30 min of drift -> two minutes of it plus anchor points",
          len(lines) <= 2 * gl.STILL_WINDOW // 1000 + 10 and sv_active(),
          "lines=%d" % len(lines))
    check("STAY: anchor on entry and every 5 min, without elevation",
          len(beats) >= 6, "beats=%d" % len(beats))
    if len(beats) >= 3:
        ts = [l.strip().split(",")[4] for l in beats[1:3]]
        gap = ((int(ts[1][9:11]) - int(ts[0][9:11])) * 3600
               + (int(ts[1][11:13]) - int(ts[0][11:13])) * 60
               + int(ts[1][13:15]) - int(ts[0][13:15]))
        check("STAY: heartbeats are STILL_HEARTBEAT apart",
              gl.STILL_HEARTBEAT // 1000 <= gap
              <= gl.STILL_HEARTBEAT // 1000 + 2, "gap=%s" % gap)
    a = gl.log["still"]["anchor"]
    check("STAY: the anchor is the mean of the drift (< 15 m off)",
          gl._dist(a[0], a[1], 47.5, 19.0) < 15.0,
          "%.1f m" % gl._dist(a[0], a[1], 47.5, 19.0))

    # excursions that turn back do not end the stay
    n_before = len(sv_lines())
    for m in (30, 55, 80, 95, 70, 40, 10):           # beyond the drift box
        sv_fix(m, 0)
    for m in (60, 120, 155, 162, 130, 60, 20):       # two samples past 150 m
        sv_fix(0, m)
    check("STAY: excursions that turn back keep the stay, write nothing",
          sv_active() is True and len(sv_lines()) == n_before,
          "active=%s new=%d" % (sv_active(), len(sv_lines()) - n_before))

    # a real departure leaves and writes the way out from the anchor
    for _ in range(5):
        sv_fix(0, 0)
    n_before = len(sv_lines())
    for i in range(1, 41):                           # 20 km/h, 5.5 m/sample
        sv_fix(i * 5.5, 0, 20.0)
    new = sv_lines()[n_before:]
    check("STAY: driving away ends the stay and keeps recording",
          sv_active() is False and len(new) >= 38,
          "active=%s new=%d" % (sv_active(), len(new)))
    check("STAY: the departure is written from the anchor, not from 150 m",
          bool(new) and sv_m(new[0]) < 15.0,
          repr(new[0] if new else None))

    # walking (1.4 m/s) is never a stay, even with the speed reading low
    sv_reset()
    for i in range(10 * 60):
        sv_fix(i * 1.4, 0, 1.0)
    check("STAY: a 10 min walk is never stationary, fully recorded",
          sv_active() is False and len(sv_lines()) >= 190,     # MIN_MOVE 3 m
          "points=%d" % len(sv_lines()))

    # walking slowly away from a stay (1 m/s) leaves past STILL_EXIT
    sv_reset()
    sv_drift(150, box=10.0, step=3.0)
    check("STAY: setup - stationary", sv_active() is True)
    left = None
    for i in range(1, 600):
        sv_fix(i * 1.0, 0, 3.6)
        if not sv_active():
            left = i * 1.0
            break
    # the first STILL_NEAR m of a slow walk pass for drift and pull the
    # anchor along, so it leaves somewhat past STILL_EXIT
    check("STAY: a slow walk away leaves just past STILL_EXIT",
          left is not None and gl.STILL_EXIT < left < gl.STILL_EXIT + 25,
          "left at %s m" % left)

    # no speed reading at all: the same, positions only
    sv_reset()
    sv_drift(200, speed=None)                # up to three windows
    check("STAY: works without a speed reading", sv_active() is True)

    # a traffic jam: two minutes within 30 m is a stay; driving on ends it
    sv_reset()
    for i in range(1, 21):
        sv_fix(i * 5.5, 0, 20.0)
    for i in range(130):
        sv_fix(110 + i * 0.15, 0, 2.0)               # crawling ~20 m
    check("STAY: a jam of two calm minutes is a stay", sv_active() is True)
    for i in range(1, 40):
        sv_fix(130 + i * 6.0, 0, 22.0)
    check("STAY: driving on after the jam ends it", sv_active() is False)

    # home network: arrive, stay, leave
    sv_reset(home=True)
    for i in range(-20, 1):                          # arriving at 20 km/h
        sv_fix(i * 5.5, 0, 20.0)
    arrived = len(sv_lines())
    check("STAY: arriving home is recorded up to the stop", arrived >= 20,
          "points=%d" % arrived)
    home_file = gl.log["file"]
    sv_drift(150)
    check("STAY: a stay on the home network closes the track",
          sv_active() and gl.log["still"]["home"] and gl.log["file"] is None
          and home_file in gl.pending())
    n_before = len(sv_lines())
    sv_drift(30 * 60)
    check("STAY: 30 min at home writes no points (no heartbeat)",
          len(sv_lines()) == n_before and gl.log["file"] is None,
          "new=%d" % (len(sv_lines()) - n_before))
    for _ in range(5):
        sv_fix(0, 0)
    for i in range(1, 41):
        sv_fix(0, i * 5.5, 20.0)
    new = gl.log["batch"]
    check("STAY: leaving home opens a new track from the anchor",
          sv_active() is False and gl.log["file"] not in (None, home_file)
          and bool(new) and sv_m(new[0]) < 15.0,
          "file=%s first=%r" % (gl.log["file"], new[0] if new else None))
    check("STAY: the new track is named after its first point",
          bool(new) and gl.log["file"].endswith(
              new[0].strip().split(",")[4].replace("T", "_")[:15] + ".trk"),
          "file=%s first=%r" % (gl.log["file"], new[0] if new else None))

    # driving past the home network without stopping is all recorded
    sv_reset(home=True)
    for i in range(300):
        sv_fix(i * 6.0, 0, 22.0)
    check("STAY: driving through home WiFi is never a stay",
          sv_active() is False and len(sv_lines()) >= 290)

    # home network lost during a stay: anchor points start, no resume
    sv_reset(home=True)
    sv_drift(150)
    check("STAY: setup - home stay", sv_active() and gl.log["still"]["home"])
    gl.wifi.update(state="offline", home=False)
    sv_drift(11 * 60)
    beats = [l for l in sv_lines() if l.split(",")[2] == ""]
    check("STAY: home WiFi lost while staying -> heartbeats, still a stay",
          sv_active() and gl.log["still"]["home"] is False
          and 2 <= len(beats) <= 4, "beats=%d" % len(beats))

    # HB: the stay's heartbeat needs no fix; GAP: windows across a fix gap;
    # ORDER: the way out stays in time order
    import calendar

    def hb_fix(east, north, hdop=1.0, nofix=False, speed=NOISE):
        """Like sv_fix; nofix leaves pos/time stale (no new GPS epoch)."""
        if speed == NOISE:
            speed = rnd.uniform(0, 22)
        _clock[0] += gl.SAMPLE
        if nofix:
            gl.log_tick()
            return
        sec = rv_sec[0]
        rv_sec[0] += 1
        gl.st.update(valid=True, date="2026-10-18",
                     time="%02d:%02d:%02d" % (8 + sec // 3600,
                                              (sec // 60) % 60, sec % 60),
                     lat=47.5 + north * M_LAT, lon=19.0 + east * M_LON,
                     alt=100.0, pos_at=_clock[0], hdop=hdop,
                     hdop_at=_clock[0], status="A", quality=1, fixtype=3,
                     speed=speed)
        gl.log_tick()

    def hb_secs(line):
        tc = line.strip().split(",")[4]
        return calendar.timegm((int(tc[0:4]), int(tc[4:6]), int(tc[6:8]),
                                int(tc[9:11]), int(tc[11:13]),
                                int(tc[13:15]), 0, 0, 0))

    # calendar arithmetic against the CPython calendar
    ok = True
    for y, m, d, hh, mi, ss in ((2026, 10, 18, 8, 0, 0), (2026, 12, 31, 23, 59, 59),
                                (2028, 2, 29, 12, 0, 0), (2100, 3, 1, 0, 0, 0),
                                (2000, 1, 1, 0, 0, 0), (2099, 12, 31, 23, 59, 59)):
        secs = calendar.timegm((y, m, d, hh, mi, ss, 0, 0, 0))
        gl.st.update(date="%04d-%02d-%02d" % (y, m, d),
                     time="%02d:%02d:%02d" % (hh, mi, ss))
        iso = "%04d-%02d-%02dT%02d:%02d:%02dZ" % (y, m, d, hh, mi, ss)
        ok = ok and gl._fix_secs() == secs and gl._stamp_at(secs)[0] == iso
    ok = ok and gl._stamp_at(calendar.timegm((2026, 12, 31, 23, 59, 59, 0, 0, 0))
                             + 1)[0] == "2027-01-01T00:00:00Z"
    check("HB: UTC date arithmetic matches the calendar (year end, leap, 2100)",
          ok)

    # HDOP too dilute for 30 min after entering: anchor points keep coming
    sv_reset()
    sv_drift(150)
    check("HB: setup - away stay", sv_active() is True)
    n0 = len([l for l in sv_lines() if l.split(",")[2] == ""])
    for _ in range(30 * 60):
        hb_fix(rnd.uniform(-15, 15), rnd.uniform(-15, 15), hdop=8.0)
    beats = [l for l in sv_lines() if l.split(",")[2] == ""]
    check("HB: HDOP > MAX_HDOP for 30 min still writes an anchor every 5 min",
          len(beats) - n0 >= 6, "beats=%d" % (len(beats) - n0))

    # no fix at all for 30 min: stamped from the last fix's time + ticks
    sv_reset()
    sv_drift(150)
    last_fix_line = sv_lines()[-1]
    t_last = rv_sec[0] - 1                      # seconds of the last fix
    for _ in range(30 * 60):
        hb_fix(0, 0, nofix=True)
    beats = [l for l in sv_lines() if l.split(",")[2] == ""]
    secs = [hb_secs(l) for l in beats]
    gaps = [b - a for a, b in zip(secs, secs[1:])]
    t0 = calendar.timegm((2026, 10, 18, 8, 0, 0, 0, 0, 0))
    check("HB: no fix for 30 min still writes an anchor every 5 min",
          len(beats) >= 7 and all(gl.STILL_HEARTBEAT // 1000 <= g
                                  <= gl.STILL_HEARTBEAT // 1000 + 2
                                  for g in gaps[1:]),
          "beats=%d gaps=%s" % (len(beats), gaps))
    check("HB: the anchor time runs on from the last GPS time",
          bool(secs) and abs(secs[-1] - (t0 + t_last + 30 * 60)) <= 300,
          "last=%s expected~%s" % (secs[-1:], t0 + t_last + 30 * 60))

    # at home there is no heartbeat, fix or not
    sv_reset(home=True)
    sv_drift(150)
    n_before = len(sv_lines())
    for _ in range(20 * 60):
        hb_fix(0, 0, nofix=True)
    check("HB: a home stay writes nothing without a fix either",
          len(sv_lines()) == n_before and gl.log["file"] is None)

    # GAP: a minute at A, fix lost for an hour, back 1.5 km away
    sv_reset()
    sv_drift(119, box=10.0, step=3.0)
    for _ in range(3600):
        hb_fix(0, 0, nofix=True)
    hb_fix(1500, 0)
    first_b = sv_lines()[-1]
    for _ in range(5):
        hb_fix(1500, 0)
    beats = [l for l in sv_lines() if l.split(",")[2] == ""]
    check("GAP: a fix gap does not prove a stay at the old place",
          sv_active() is False and beats == [],
          "active=%s beats=%s" % (sv_active(), beats))
    check("GAP: the first fix after the gap is kept",
          abs(float(first_b.split(",")[1]) - (19.0 + 1500 * M_LON)) < 1e-4,
          repr(first_b))

    # GAP: the same place after a gap keeps its windows
    sv_reset()
    sv_drift(70, box=10.0, step=3.0)
    prev_before = gl.log["still"]["prev"]
    for _ in range(45):
        hb_fix(0, 0, nofix=True)
    hb_fix(2, 2)
    check("GAP: a gap at the same place keeps the previous window",
          prev_before is not None and gl.log["still"]["prev"] is prev_before)
    sv_drift(60, box=10.0, step=3.0)
    check("GAP: ... and the stay is entered on time",
          sv_active() is True)

    # ORDER: an anchor point due while the car drives off
    sv_reset()
    sv_drift(150, box=10.0, step=3.0)
    gl.log["still"]["hb_at"] = _clock[0] - gl.STILL_HEARTBEAT + 4000
    for i in range(1, 41):
        hb_fix(i * 6.0, 0, speed=22.0)
    lines = sv_lines()
    ts = [hb_secs(l) for l in lines]
    check("ORDER: the track stays in time order through a departure",
          all(b >= a for a, b in zip(ts, ts[1:])) and sv_active() is False,
          "steps back=%d" % sum(1 for a, b in zip(ts, ts[1:]) if b < a))
    out = [l for l in lines if sv_m(l) > 15]
    check("ORDER: the way out is still written from near the anchor",
          bool(out) and sv_m(out[0]) < 60.0,
          repr(out[0] if out else None))

    # EXIT: a frozen multipath jump is not a departure; a slow walk still is
    sv_reset()
    sv_drift(150, box=15.0, step=5.0)
    check("EXIT: setup - away stay", sv_active() is True)
    n0 = len(sv_lines())
    for _ in range(3):
        sv_fix(200.0, 0)                           # frozen 200 m jump, 3 s
    sv_drift(5, box=15.0, step=5.0)
    check("EXIT: a 3 s frozen jump to 200 m is not a departure",
          sv_active() is True and len(sv_lines()) == n0,
          "active=%s new=%d" % (sv_active(), len(sv_lines()) - n0))
    for i in range(9):
        sv_fix(200.0 + i * 1.5, 0)                 # jump that creeps 1.5 m/s
    check("EXIT: a creeping jump below STILL_STEP needs STILL_EXIT_SLOW samples",
          sv_active() is True)
    sv_fix(220.0, 0)
    check("EXIT: ... and leaves on the 10th sample out there",
          sv_active() is False)
    sv_reset()
    sv_drift(150, box=15.0, step=5.0)
    for i in range(1, 60):
        sv_fix(i * 6.0, 0, 22.0)
    check("EXIT: a car (6 m/s) leaves after STILL_EXIT_N gaining samples",
          sv_active() is False and len([l for l in sv_lines()
                                        if sv_m(l) > 150]) >= 25)

    # ARRIVE: entering a stay closes the arrival track on any network; the
    # anchor file rotates every STILL_FILE while connected
    sv_reset()
    gl.wifi.update(state="connected", home=False)
    for i in range(300):                            # 5 min drive at 6 m/s
        sv_fix(-6.0 * (300 - i), 0, 22.0)
    arrival = gl.log["file"]
    sv_drift(150, box=15.0, step=5.0)
    check("ARRIVE: the arrival track is closed when the stay begins",
          sv_active() and arrival in gl.pending() and gl.log["file"] != arrival,
          "file=%s pending=%s" % (gl.log["file"], gl.pending()))
    anchor_file = gl.log["file"]
    sv_drift(3600, box=15.0, step=5.0)
    check("ARRIVE: the anchor file rotates every STILL_FILE on WiFi",
          anchor_file in gl.pending() and len(gl.pending()) >= 3,
          "pending=%d" % len(gl.pending()))
    check("ARRIVE: files opened during the stay hold anchor points only",
          all(l.split(",")[2] == "" for p in gl.pending() if p >= anchor_file
              for l in open(p) if l.count(",") == 4))
    sv_reset()
    sv_drift(150, box=15.0, step=5.0)               # offline
    af, n_pend = gl.log["file"], len(gl.pending())
    sv_drift(3600, box=15.0, step=5.0)
    check("ARRIVE: offline the anchor file is kept open (nothing to upload to)",
          gl.log["file"] == af and len(gl.pending()) == n_pend)

    # HBFILE: a heartbeat that opens a file names it after its own time, and
    # the file closes once the home network is back, fix or not
    sv_reset(home=True)
    sv_drift(150, box=15.0, step=5.0)
    for _ in range(3600):
        hb_fix(0, 0, nofix=True)                    # an hour without a fix
    gl.wifi.update(state="offline", home=False)
    for _ in range(10):
        hb_fix(0, 0, nofix=True)
    f = gl.log["file"]
    b = ([l for l in open(f) if l.count(",") == 4]
         if f and os.path.exists(f) else []) + list(gl.log["batch"])
    check("HBFILE: heartbeat file is named after the anchor point's time",
          f is not None and b and os.path.basename(f)[:15]
          == b[0].strip().split(",")[4].replace("T", "_")[:15],
          "file=%s line=%r" % (f, b[:1]))
    gl.wifi.update(state="connected", home=True)
    for _ in range(5):
        hb_fix(0, 0, nofix=True)
    check("HBFILE: back on home WiFi the file closes without a fix",
          gl.log["file"] is None and gl.log["batch"] == [] and f in gl.pending())

    # BAND: settled 50-150 m from the anchor (out of the car, into the
    # building) moves the anchor there; the heartbeat goes on
    def beats_of(lines):
        return [l for l in lines if l.split(",")[2] == ""]

    sv_reset()
    sv_drift(150, box=10.0, step=3.0)
    check("BAND: setup - away stay", sv_active() is True)
    n0 = len(beats_of(sv_lines()))
    for _ in range(30 * 60):
        sv_fix(100 + rnd.uniform(-10, 10), rnd.uniform(-10, 10))
    new = beats_of(sv_lines())[n0:]
    a = gl.log["still"]["anchor"]
    a_off = gl._dist(a[0], a[1], 47.5, 19.0 + 100 * M_LON)
    check("BAND: 30 min settled 100 m away moves the anchor there",
          sv_active() is True and a_off < 15.0, "anchor %.1f m off" % a_off)
    check("BAND: ... and writes anchor points there every 5 min",
          len(new) >= 5 and all(80 < sv_m(l) < 120 for l in new[1:]),
          "beats=%d at %s m" % (len(new), [int(sv_m(l)) for l in new]))
    n0 = len(sv_lines())
    for i in range(1, 60):
        sv_fix(100 + i * 6.0, 0, 22.0)
    out = sv_lines()[n0:]
    check("BAND: driving off later leaves from the new anchor",
          sv_active() is False and bool(out) and 85 < sv_m(out[0]) < 130,
          "active=%s first=%s" % (sv_active(), out[:1]))
    # a visit to 100 m shorter than two windows does not move it
    sv_reset()
    sv_drift(150, box=10.0, step=3.0)
    for _ in range(90):
        sv_fix(100 + rnd.uniform(-10, 10), rnd.uniform(-10, 10))
    sv_drift(60, box=10.0, step=3.0)
    a = gl.log["still"]["anchor"]
    check("BAND: 90 s at 100 m that comes back keeps the anchor",
          sv_active() is True and gl._dist(a[0], a[1], 47.5, 19.0) < 15.0,
          "%.1f m" % gl._dist(a[0], a[1], 47.5, 19.0))

    sv_reset()
    sv_drift(150, box=10.0, step=3.0)
    for _ in range(210):                           # 3.5 min shifted 100 m
        sv_fix(100 + rnd.uniform(-10, 10), rnd.uniform(-10, 10))
    sv_drift(60, box=10.0, step=3.0)
    a = gl.log["still"]["anchor"]
    check("BAND: a 3.5 min shift (a multipath episode) keeps the anchor",
          sv_active() is True and gl._dist(a[0], a[1], 47.5, 19.0) < 15.0,
          "%.1f m" % gl._dist(a[0], a[1], 47.5, 19.0))

    sv_reset()
    sv_drift(150, box=10.0, step=3.0)
    for secs, (e, n) in ((125, (80, 0)), (60, (0, 80)), (125, (80, 0)),
                         (60, (0, 80)), (125, (80, 0))):
        for _ in range(secs):                       # recurring, not steady
            sv_fix(e + rnd.uniform(-5, 5), n + rnd.uniform(-5, 5))
    a = gl.log["still"]["anchor"]
    check("BAND: recurring 80 m episodes (hits not consecutive) keep the anchor",
          sv_active() is True and gl._dist(a[0], a[1], 47.5, 19.0) < 15.0,
          "%.1f m" % gl._dist(a[0], a[1], 47.5, 19.0))

    # FLASH: anchor points written without a fix reach flash every FLUSH
    sv_reset()
    sv_drift(150, box=10.0, step=3.0)
    for _ in range(2 * 3600):
        hb_fix(0, 0, nofix=True)
    on_flash = 0
    for n in os.listdir(rv_dir):
        if n.endswith(".trk"):
            on_flash += len(beats_of(l for l in open(os.path.join(rv_dir, n))
                                     if l.count(",") == 4))
    check("FLASH: 2 h without a fix -> the anchor points are on flash",
          on_flash >= 24 and len(gl.log["batch"]) <= 1,
          "flash=%d ram=%d" % (on_flash, len(gl.log["batch"])))

    # ONEFILE: a fix that comes back after a long gap keeps the stay's file
    sv_reset()
    sv_drift(150, box=10.0, step=3.0)
    stay_file = gl.log["file"]
    for _ in range(18):                             # 3 h, fix every 10 min
        for _ in range(9 * 60):
            hb_fix(0, 0, nofix=True)
        sv_drift(60, box=10.0, step=3.0)
    gl.flush()
    trk = sorted(os.path.join(rv_dir, n) for n in os.listdir(rv_dir)
                 if n.endswith(".trk"))
    check("ONEFILE: 3 h offline stay with fix gaps stays in one file",
          sv_active() and gl.log["file"] == stay_file
          and [p for p in trk if p >= stay_file] == [stay_file],
          "files=%s" % [os.path.basename(p) for p in trk])

    # RETRY: a failing flush at home is retried every FLUSH, not every tick
    sv_reset(home=True)
    real_flush, calls = gl.flush, [0]

    def failing_flush(*a):
        calls[0] += 1
        gl.log["last_flush"] = _clock[0]
    gl.flush = failing_flush
    try:
        for i in range(-180, 1):
            sv_fix(i * 8.3, 0, 30.0)
        sv_drift(150, box=10.0, step=3.0)
        c0 = calls[0]
        for _ in range(500):                        # 10 s of 20 ms loops
            _clock[0] += 20
            gl.log_tick()
        tries = calls[0] - c0
    finally:
        gl.flush = real_flush
    check("RETRY: a failing flush in a home stay is not retried every tick",
          sv_active() and gl.log["file"] is not None and tries <= 1,
          "flushes in 10 s=%d" % tries)
    for _ in range(gl.FLUSH // 1000 + 2):
        hb_fix(0, 0, nofix=True)
    check("RETRY: ... and once flash works the home track closes",
          gl.log["file"] is None and gl.log["batch"] == [])

    # FROZEN: a jump frozen out past STILL_EXIT gains nothing, however long
    sv_reset()
    sv_drift(150, box=15.0, step=5.0)
    n0 = len(sv_lines())
    for _ in range(30):
        sv_fix(200.0 + rnd.uniform(-0.5, 0.5), 0)
    sv_drift(5, box=15.0, step=5.0)
    check("FROZEN: a 30 s frozen jump to 200 m is not a departure",
          sv_active() is True and len(sv_lines()) == n0,
          "active=%s new=%d" % (sv_active(), len(sv_lines()) - n0))

    # HOLD: the drift that proves a stay is held in RAM, so a boot (or a new
    # fix) at home uploads nothing of it; a real arrival is still kept
    import math
    for home in (True, False):
        sv_reset(home=home)
        for _ in range(30 * 60):
            e, n = rnd.gauss(0, 25), rnd.gauss(0, 25)
            if rnd.random() < 0.05:                 # multipath jumps
                ang = rnd.uniform(0, 2 * math.pi)
                e, n = e + 110 * math.cos(ang), n + 110 * math.sin(ang)
            sv_fix(e, n)
        gl.flush()
        drift = [l for l in sv_lines() if l.split(",")[2] != ""]
        check("HOLD: 30 min of drift from boot leaves no drift point (home=%s)"
              % home, sv_active() is True and drift == [],
              "drift lines=%d" % len(drift))
    sv_reset(home=True)
    for i in range(-300, 1):                        # 30 km/h, stop at 0
        sv_fix(i * 8.3, 0, 30.0)
    sv_drift(200, box=10.0, step=3.0)
    gl.flush()
    way_in = [sv_m(l) for l in sv_lines()]
    check("HOLD: the way in is kept up to the stay",
          sv_active() is True and len(way_in) >= 240
          and min(way_in) < gl.STILL_NEAR + 15,
          "points=%d closest=%.0f m" % (len(way_in), min(way_in or [0])))
    # a window that ends the drive and starts the stop, and does not pair
    # with the next one, is no longer held: at most its tail reaches flash
    at_stop = [m for m in way_in if m < 15]
    check("HOLD: ... and at most one window of the drift at the stop",
          len(at_stop) <= gl.STILL_WINDOW // 1000,
          "drift lines at the stop=%d" % len(at_stop))

    # MIDNIGHT: a GGA past midnight whose RMC was lost is not dated a day back
    gl.reset()
    epoch("235959.00", date="181026")
    epoch("235959.20", date="181026")
    gl.handle(nmea("GPGGA,000000.00,4730.000,N,01900.000,E,1,08,1.0,100.0,"
                   "M,40.0,M,,"))                   # its RMC was lost
    check("MIDNIGHT: setup - fix at 23:59:59",
          bool(gl.has_fix(_clock[0])) and gl.st["date"] == "2026-10-18"
          and gl.st["time"] == "23:59:59")
    _clock[0] += 1000
    epoch("000001.00", date="191026")
    check("MIDNIGHT: the second past midnight without an RMC is no fix",
          not gl.has_fix(_clock[0]) and gl._fix_secs() is None
          and gl.st["time"] == "00:00:00")
    _clock[0] += 1000
    epoch("000002.00", date="191026")
    s = gl._sample_line(_clock[0], None, None)
    check("MIDNIGHT: the next second brings the new date back",
          bool(gl.has_fix(_clock[0])) and s is not None
          and s[0] == "2026-10-19T00:00:01Z", repr(s))
    gl.reset()
    epoch("235959.00", date="181026")
    epoch("000000.00", date="191026")               # RMC first: new date
    check("MIDNIGHT: the last second of a day keeps its own date",
          gl.st["time"] == "23:59:59" and gl.st["date"] == "2026-10-18")

    # SPARSE: indoors only a few usable fixes a minute; a window waits for
    # STILL_MIN_FIXES (up to STILL_WINDOW_MAX), so the stay is still proven
    def sparse(minutes, per_min, east_of=lambda s: 0.0):
        for m in range(minutes):
            on = set(rnd.sample(range(60), per_min))
            for k in range(60):
                if k in on:
                    sv_fix(east_of(m * 60 + k) + rnd.gauss(0, 20),
                           rnd.gauss(0, 20))
                else:
                    hb_fix(0, 0, nofix=True)
    sv_reset(home=True)
    sparse(60, 3)
    gl.flush()
    check("SPARSE: 3 usable fixes a minute at home -> a stay, no drift track",
          sv_active() and len(sv_lines()) <= 5,
          "active=%s lines=%d" % (sv_active(), len(sv_lines())))
    leaks = []
    for seed in range(5):                            # home WiFi connected
        sv_reset(home=True)
        rnd.seed(seed)
        sparse(14, 3)
        on_flash = 0
        for n in os.listdir(rv_dir):
            if n.endswith(".trk"):
                on_flash += len([l for l in open(os.path.join(rv_dir, n))
                                 if l.count(",") == 4])
        leaks.append((sv_active(), on_flash))
    rnd.seed(1016)
    check("SPARSE: no drift reaches flash while the stay is proven (WiFi on)",
          all(a2 and f == 0 for a2, f in leaks), "runs=%s" % leaks)
    # CLOSE: closing a track for upload keeps the held drift out of it
    sv_reset()
    gl.start_file("20261018_080000")
    gl.log["batch"][:] = ["47.500000,19.000000,100.0,1.0,20261018T0800%02dZ\n"
                          % k for k in (0, 5)]
    gl.flush()
    old = gl.log["file"]
    gl.log["still"].update(win_at=_clock[0], win_tc="20261018T080010Z")
    gl.log["batch"][:] = ["47.500100,19.000000,100.0,1.0,20261018T0800%02dZ\n"
                          % k for k in (10, 15)]
    closed = gl._close_file()
    with open(old) as fh:
        old_pts = [l for l in fh if l.count(",") == 4]
    check("CLOSE: the closed track has only lines before the hold",
          closed and len(old_pts) == 2 and old in gl.pending())
    check("CLOSE: the held lines move on to a file named after the first",
          gl.log["file"].endswith("20261018_080010.trk")
          and len(gl.log["batch"]) == 2 and not gl.log["created"])
    # GARAGE: a window at A, no fix for 2 min, then a stay 120 m away at B:
    # the stay is anchored at B, not between the two
    sv_reset()
    for _ in range(40):
        sv_fix(rnd.uniform(-5, 5), rnd.uniform(-5, 5))
    for _ in range(120):
        hb_fix(0, 0, nofix=True)
    for _ in range(200):
        sv_fix(120 + rnd.uniform(-5, 5), rnd.uniform(-5, 5))
    a = gl.log["still"]["anchor"]
    off = gl._dist(a[0], a[1], 47.5, 19.0 + 120 * M_LON) if a else None
    check("GARAGE: a stay after a gap 120 m away is anchored there",
          sv_active() and off is not None and off < 15.0, "off=%s" % off)

    # SEG: a short fix loss puts the segment break before the first line
    # after it, not before lines recorded earlier and not yet flushed
    sv_reset()
    for i in range(10):
        sv_fix(i * 6.0, 0, 22.0)                    # driving, in RAM
    for _ in range(20):
        hb_fix(0, 0, nofix=True)                    # 20 s without a fix
    for i in range(10, 20):
        sv_fix(i * 6.0, 0, 22.0)
    gl.flush()
    with open(gl.log["file"]) as fh:
        body = [l for l in fh if not l.startswith("#")]
    mark = body.index("-\n") if "-\n" in body else None
    check("SEG: the break lands between the lines before and after the loss",
          mark is not None and 8 <= mark <= 10 and body.count("-\n") == 1,
          "mark=%s lines=%d" % (mark, len(body)))

    # FIXGAP: a fix back after a long gap closes the track; what a stay
    # still being proven holds moves on, nothing of it is written
    sv_reset()
    gl.wifi.update(state="offline")
    for i in range(20):
        sv_fix(i * 6.0, 0, 22.0)                    # a drive
    gl.flush()
    trip = gl.log["file"]
    for _ in range(40):
        sv_fix(120 + rnd.uniform(-5, 5), rnd.uniform(-5, 5))   # held
    for _ in range(gl.NEW_FILE_GAP + 10):
        hb_fix(0, 0, nofix=True)
    held_before = len(gl.log["batch"])
    _clock[0] += gl.FLUSH
    sv_fix(120, 0)
    with open(trip) as fh:
        trip_pts = [l for l in fh if l.count(",") == 4]
    check("FIXGAP: the trip closes with only its own lines",
          trip in gl.pending() and len(trip_pts) == 20
          and gl.log["file"] != trip, "pts=%d file=%s" % (len(trip_pts),
                                                         gl.log["file"]))
    check("FIXGAP: the held drift moved on in RAM, not to flash",
          held_before > 0 and len(gl.log["batch"]) >= held_before)
    sv_reset()
    gl.wifi.update(state="offline")
    for i in range(20):
        sv_fix(i * 6.0, 0, 22.0)
    for _ in range(gl.NEW_FILE_GAP + 10):
        hb_fix(0, 0, nofix=True)
    gl.log["last_flush"] = _clock[0]                # a flush just happened
    gl.log["batch"].append("47.6,19.1,1.0,1.0,20261018T235959Z\n")
    sv_fix(500, 0, 22.0)
    check("FIXGAP: right after a flush the track stays, with a new segment",
          gl.log["new_seg"] is True and gl.log["prev"] is not None)

    sv_reset()
    sparse(20, 3, east_of=lambda s: s * 1.4)
    check("SPARSE: a walk with 3 fixes a minute is never a stay",
          not sv_active() and len(sv_lines()) >= 40,
          "active=%s lines=%d" % (sv_active(), len(sv_lines())))

    # SLEEP: the radio is on only until everything is uploaded, then off for
    # WIFI_SLEEP; a round that finds nothing also sleeps; a new stay wakes it
    def wl():
        return gl.wifi["wlan"]

    def both(seconds, east=0.0, box=10.0, speed=NOISE):
        for _ in range(seconds):
            sv_fix(east + rnd.uniform(-box, box), rnd.uniform(-box, box), speed)
            gl.wifi_tick()

    def beats_now():
        return len([l for l in sv_lines() if l.split(",")[2] == ""])

    sv_reset()
    w, calls, out = rv_start_wifi(rv_env, [("home", -40)], ["home"])
    make_trk(os.path.join(rv_dir, "20261018_070000.trk"), 3)
    gl.up["left"] = 3
    _clock[0] += gl.WIFI_IDLE
    gl.wifi_tick()
    check("SLEEP: a track waiting for upload keeps the WiFi on",
          gl.wifi["state"] == "connected" and wl().active())
    _posts[:] = []
    _post_code[0] = 201
    pump()
    _clock[0] += gl.WIFI_IDLE
    gl.wifi_tick()
    check("SLEEP: once everything is uploaded the radio goes off",
          len(_posts) == 1 and gl.wifi["state"] == "sleep"
          and not wl().active() and gl.online() is False,
          "posts=%d state=%s" % (len(_posts), gl.wifi["state"]))
    n_conn = len(WLAN.connects)
    _clock[0] += gl.WIFI_SLEEP - 1000
    gl.wifi_tick()
    check("SLEEP: nothing is tried for WIFI_SLEEP",
          gl.wifi["state"] == "sleep" and len(WLAN.connects) == n_conn)
    for _ in range(30):
        _clock[0] += gl.WIFI_IDLE
        gl.wifi_tick()
    check("SLEEP: after the hour, with nothing to upload, it sleeps on",
          gl.wifi["state"] == "sleep" and not wl().active()
          and len(WLAN.connects) == n_conn)
    make_trk(os.path.join(rv_dir, "20261018_080000.trk"), 3)
    _clock[0] += gl.WIFI_IDLE
    gl.wifi_tick()
    drain_wifi()
    check("SLEEP: ... and searches once a track waits for upload",
          gl.wifi["state"] == "connected" and wl().active()
          and len(WLAN.connects) > n_conn)
    os.remove(os.path.join(rv_dir, "20261018_080000.trk"))

    # WEB: at home the status page keeps the radio on; away it sleeps as
    # before, but an hourly round finds home again without points
    gl.WEB = True
    try:
        sv_reset()
        w, calls, out = rv_start_wifi(rv_env, [("home", -40)], ["home"])
        for _ in range(5):
            _clock[0] += gl.WIFI_IDLE
            gl.wifi_tick()
        web_home = gl.wifi["state"] == "connected" and wl().active()
        sv_reset()
        w, calls, out = rv_start_wifi(rv_env, [("phone", -40)], ["phone"])
        away_conn = (gl.wifi["state"] == "connected"
                     and gl.wifi["ssid"] == "phone" and not gl.wifi["home"])
        _clock[0] += gl.WIFI_IDLE
        gl.wifi_tick()
        web_away = gl.wifi["state"] if away_conn else "not connected"
        n_conn = len(WLAN.connects)
        _clock[0] += gl.WIFI_SLEEP + gl.WIFI_IDLE
        gl.wifi_tick()
        drain_wifi()
        web_hourly = len(WLAN.connects) > n_conn
    finally:
        gl.WEB = False
    check("WEB: at home, everything uploaded, the radio stays connected",
          web_home, "state=%s" % gl.wifi["state"])
    check("WEB: on a non-home network it still sleeps once uploaded",
          web_away == "sleep", web_away)
    check("WEB: after WIFI_SLEEP a round runs even with nothing to upload",
          web_hourly)

    # WEB server: GET / and /status.json answered from the main loop
    class FakeConn:
        def __init__(self, req, fail=False):
            self.req, self.fail, self.sent, self.closed = req, fail, b"", False
            self.timeout = None

        def settimeout(self, t):
            self.timeout = t

        def recv(self, n):
            if self.fail:
                raise OSError(104)
            return self.req[:n]

        def sendall(self, b):
            self.sent += b

        def close(self):
            self.closed = True

    class FakeListen:
        def __init__(self):
            self.waiting, self.closed, self.bound = [], False, None
            self.opts, self.blocking, self.backlog = [], True, None

        def setsockopt(self, *a):
            self.opts.append(a)

        def bind(self, addr):
            self.bound = addr

        def listen(self, n):
            self.backlog = n

        def setblocking(self, b):
            self.blocking = b

        def accept(self):
            if not self.waiting:
                raise OSError(11)
            return self.waiting.pop(0), ("192.168.1.209", 50000)

        def close(self):
            self.closed = True

    class FakeSocketMod:
        AF_INET, SOCK_STREAM, SOL_SOCKET, SO_REUSEADDR = 2, 1, 1, 4
        made = []

        @classmethod
        def socket(cls, *a):
            cls.made.append(FakeListen())
            return cls.made[-1]

        @staticmethod
        def getaddrinfo(host, port):
            return [(2, 1, 0, "", (host, port))]

    real_socket = gl.socket
    gl.socket = FakeSocketMod
    gl.WEB = True
    try:
        gl._web_close()
        gl.wifi.update(state="connected", ip="192.168.1.60", ssid="<b>home",
                       rssi=-50)
        gl.log["points"] = 42
        gl.up["left"] = 7
        gl.up["sent"] = 35
        gl._diag_write("boot cause 1 mhz 80 mem 1")
        gl._diag_write("up 60 utc - nmea 300/0 <x>")
        gl.web_tick()
        lst = FakeSocketMod.made[-1]
        opened = (lst.bound == ("0.0.0.0", 80) and lst.blocking is False
                  and gl._web_sock is lst)
        reqs = [FakeConn(b"GET / HTTP/1.1\r\nHost: x\r\n\r\n"),
                FakeConn(b"GET /status.json HTTP/1.1\r\n\r\n"),
                FakeConn(b"GET /nope HTTP/1.1\r\n\r\n"),
                FakeConn(b"", fail=True)]
        for c in reqs:
            lst.waiting.append(c)
            gl.web_tick()
        gl.web_tick()                        # nobody waiting: no error
        head, _, html = reqs[0].sent.partition(b"\r\n\r\n")
        jhead, _, jbody = reqs[1].sent.partition(b"\r\n\r\n")
        try:
            jd = json.loads(jbody)
        except ValueError:
            jd = {}
        gl.wifi["state"] = "sleep"
        gl.web_tick()
        closed_on_sleep = lst.closed and gl._web_sock is None
        gl.WEB = False
        gl.wifi["state"] = "connected"
        n_made = len(FakeSocketMod.made)
        gl.web_tick()
        off_no_socket = len(FakeSocketMod.made) == n_made
        envs = []
        for v in ("0", "1", "maybe"):
            gl.WEB = True
            gl._web_env({"WEB": v})
            envs.append(gl.WEB)
    finally:
        gl._web_close()
        gl.socket = real_socket
        gl.WEB = False
        gl.wifi["state"] = "sleep"
    check("WEB: listens on port 80, non-blocking, while connected", opened)
    check("WEB: GET / is the HTML page with points, upload and diag lines",
          head.startswith(b"HTTP/1.0 200 OK") and b"text/html" in head
          and (b"Content-Length: %d" % len(html)) in head
          and b"Logged since boot</td><td>42<" in html
          and b"Waiting for upload</td><td>7<" in html
          and b"boot cause 1" in html and reqs[0].closed
          and reqs[0].timeout == gl.WEB_TIMEOUT, reqs[0].sent[:300])
    check("WEB: SSID and diag text are HTML-escaped",
          b"&lt;b&gt;home" in html and b"&lt;x&gt;" in html
          and b"<b>home" not in html)
    check("WEB: /status.json has the same values and the diag tail",
          jhead.startswith(b"HTTP/1.0 200 OK") and jd.get("points") == 42
          and jd.get("left") == 7 and jd.get("sent") == 35
          and "fix" in jd and "sats" in jd
          and jd.get("diag", [""])[-1].startswith("up 60"), repr(jd)[:300])
    check("WEB: unknown path 404, a failing client is closed, no raise",
          reqs[2].sent.startswith(b"HTTP/1.0 404") and reqs[3].closed
          and reqs[3].sent == b"")
    check("WEB: the server closes when the radio sleeps; WEB=0 opens none",
          closed_on_sleep and off_no_socket)
    check("WEB: .env WEB 0/1, anything else keeps the setting",
          envs == [False, True, True], repr(envs))
    import inspect
    check("WEB: _loop serves the page; wifi_stop closes it",
          "web_tick()" in inspect.getsource(gl._loop)
          and "_web_close()" in inspect.getsource(gl.wifi_stop))

    # OFF: while uploads back off for WIFI_OFF_WAIT or more (server down,
    # 401) the radio goes off until the retry is due, then a round
    # reconnects and the upload goes on; a short backoff keeps it on
    sv_reset()
    w, calls, out = rv_start_wifi(rv_env, [("home", -40)], ["home"])
    off_trk = os.path.join(rv_dir, "20261018_100000.trk")
    make_trk(off_trk, 3)
    gl.up["left"] = 3
    _posts[:] = []
    _post_code[0] = 503
    pump()                                           # backoff 15 s: on
    _clock[0] += gl.WIFI_IDLE
    gl.wifi_tick()
    check("OFF: a short upload backoff keeps the radio on",
          len(_posts) == 1 and gl.up["state"] == "err 503"
          and gl.wifi["state"] == "connected" and wl().active(),
          "state=%s up=%s" % (gl.wifi["state"], gl.up["state"]))
    for _ in range(3):                               # 30 s, 60 s, 120 s
        _clock[0] = gl.up["next_at"]
        gl.upload_tick()
    wait = gl.up["next_at"] - _clock[0]
    _clock[0] += gl.WIFI_IDLE
    gl.wifi_tick()
    check("OFF: a backoff of WIFI_OFF_WAIT or more turns the radio off",
          wait >= gl.WIFI_OFF_WAIT and gl.wifi["state"] == "sleep"
          and not wl().active()
          and gl.wifi["wake_at"] == gl.up["next_at"],
          "wait=%s state=%s" % (wait, gl.wifi["state"]))
    n_conn = len(WLAN.connects)
    _clock[0] = gl.wifi["wake_at"] - 2 * gl.WIFI_IDLE
    gl.wifi_tick()
    check("OFF: it stays off until the retry is due",
          gl.wifi["state"] == "sleep" and len(WLAN.connects) == n_conn)
    _post_code[0] = 201
    _clock[0] = gl.wifi["wake_at"] + 1
    gl.wifi_tick()
    drain_wifi()
    gl.upload_tick()
    _clock[0] += gl.WIFI_IDLE
    gl.wifi_tick()
    check("OFF: when the retry is due a round connects, uploads and sleeps",
          len(WLAN.connects) > n_conn and len(_posts) == 5
          and _posts[-1]["data"].count(b'"type":"Feature"') == 3
          and gl.up["state"] == "ok" and gl.wifi["state"] == "sleep"
          and gl.wifi["wake_at"] is None,
          "posts=%d up=%s state=%s" % (len(_posts), gl.up["state"],
                                        gl.wifi["state"]))

    # CPU: 80 MHz in the loop, CPU_MHZ_UPLOAD around a POST, back after
    sv_reset()
    w, calls, out = rv_start_wifi(rv_env, [("home", -40)], ["home"])
    check("CPU: wifi_start sets CPU_MHZ (default 80 MHz)",
          _freqs[-1] == gl.CPU_MHZ * 1000000 and gl.CPU_MHZ == 80,
          "freqs=%s" % _freqs[-3:])
    make_trk(off_trk, 2)
    gl.up["left"] = 2
    _posts[:] = []
    n0 = len(_freqs)
    pump()
    check("CPU: a POST runs at CPU_MHZ_UPLOAD and the loop clock comes back",
          len(_posts) == 1 and _freqs[n0:] == [gl.CPU_MHZ_UPLOAD * 1000000,
                                               gl.CPU_MHZ * 1000000],
          "freqs=%s" % _freqs[n0:])
    saved_cpu = gl.CPU_MHZ, gl.CPU_MHZ_UPLOAD
    try:
        out = []
        real_print = gl.print if hasattr(gl, "print") else None
        gl.print = lambda *x, **k: out.append(" ".join(str(y) for y in x))
        saved_c6 = gl.IS_C6
        try:
            gl._freq_bad[:] = []
            gl._set_freq(100)
            gl._set_freq(100)
            once = sum(1 for o in out if o.startswith("cpu freq 100")) == 1
            gl.IS_C6 = True
            c6 = [gl._mhz_env({"CPU_MHZ": v}, "CPU_MHZ", 80)
                  for v in ("40", "240", "160", "80", "0")]
        finally:
            gl.IS_C6 = saved_c6
            if real_print is None:
                del gl.print
            else:
                gl.print = real_print
        check("CPU: an unsupported clock is reported once, not per POST",
              once, repr(out))
        check("CPU: on a C6 only 80/160 (or 0) are accepted from .env",
              c6 == [80, 80, 160, 80, 0], repr(c6))
        a = gl._mhz_env({"CPU_MHZ": "160  # fast"}, "CPU_MHZ", 80)
        b = gl._mhz_env({"CPU_MHZ": "0"}, "CPU_MHZ", 80)
        c = gl._mhz_env({"CPU_MHZ": "100"}, "CPU_MHZ", 80)
        d = gl._mhz_env({}, "CPU_MHZ", 80)
        n0 = len(_freqs)
        gl._set_freq(0)
        gl._set_freq(100)                            # unsupported: left alone
        gl._set_freq(_freqs[-1] // 1000000)          # already there: no call
        same = len(_freqs) == n0
    finally:
        gl.CPU_MHZ, gl.CPU_MHZ_UPLOAD = saved_cpu
    check("CPU: .env CPU_MHZ 160 / 0 taken, 100 and missing keep the default",
          (a, b, c, d) == (160, 0, 80, 80) and same,
          "%s %s %s %s same=%s" % (a, b, c, d, same))

    # a round that finds no network: nothing to upload -> sleep; with a
    # track waiting -> the usual backoff, 15 s doubling to 10 min, until a
    # network connects
    w, calls, out = rv_start_wifi(rv_env, [], [])
    check("SLEEP: no network and nothing to upload -> radio off",
          gl.wifi["state"] == "sleep" and not wl().active(),
          "state=%s" % gl.wifi["state"])
    make_trk(os.path.join(rv_dir, "20261018_090000.trk"), 3)
    w, calls, out = rv_start_wifi(rv_env, [], [])
    n_conn = len(WLAN.connects)
    check("SLEEP: no network with a track waiting -> backoff, radio off",
          gl.wifi["state"] == "not found" and not wl().active()
          and gl.wifi["backoff"] == 2 * gl.WIFI_RETRY,
          "state=%s backoff=%s" % (gl.wifi["state"], gl.wifi["backoff"]))
    rounds, on_between = [], 0
    for _ in range(40 * 60 // 5):                  # 40 minutes
        _clock[0] += 5000
        before = len(WLAN.connects)
        gl.wifi_tick()
        drain_wifi()
        if len(WLAN.connects) > before:
            rounds.append(_clock[0])
        on_between += bool(wl().active())
    check("SLEEP: ... the radio is off between the rounds",
          on_between == 0, "ticks with radio on=%d" % on_between)
    gaps = [(b2 - a2) // 1000 for a2, b2 in zip(rounds, rounds[1:])]
    check("SLEEP: ... retried with growing gaps up to WIFI_BACKOFF_MAX",
          len(rounds) >= 5 and gaps == sorted(gaps)
          and max(gaps) <= gl.WIFI_BACKOFF_MAX // 1000 + 60, "gaps=%s" % gaps)
    WLAN.scan_aps, WLAN.ok = [("home", -40)], {"home"}
    for _ in range(gl.WIFI_BACKOFF_MAX // 5000 + 2):
        _clock[0] += 5000
        gl.wifi_tick()
        drain_wifi()
        if gl.wifi["state"] == "connected":
            break
    check("SLEEP: ... until a network connects",
          gl.wifi["state"] == "connected")
    os.remove(os.path.join(rv_dir, "20261018_090000.trk"))

    # a stay at home while the radio sleeps: still a home stay, no anchor
    sv_reset()
    w, calls, out = rv_start_wifi(rv_env, [("home", -40)], ["home"])
    both(15)
    check("SLEEP: the drift held for a stay being proven keeps the WiFi on",
          gl.wifi["state"] == "connected")
    both(150)
    check("SLEEP: a home stay, all uploaded -> asleep as home",
          sv_active() and gl.log["still"]["home"]
          and gl.wifi["state"] == "sleep" and gl.wifi["sleep_home"]
          and gl.at_home(), "state=%s sleep_home=%s" % (
              gl.wifi["state"], gl.wifi["sleep_home"]))
    sv_reset()
    w, calls, out = rv_start_wifi(rv_env, [("home", -40)], ["home"])
    gl._wifi_off(wl(), "sleep")                 # asleep before any stay
    check("SLEEP: setup - asleep, no stay yet: not home",
          gl.wifi["state"] == "sleep" and not gl.at_home())
    n_conn = len(WLAN.connects)
    both(150)
    check("SLEEP: a new stay wakes the WiFi, finds home, sleeps as home",
          sv_active() and gl.log["still"]["home"]
          and len(WLAN.connects) > n_conn and gl.wifi["state"] == "sleep"
          and gl.wifi["sleep_home"] and gl.at_home(),
          "state=%s home=%s sleep_home=%s" % (
              gl.wifi["state"], gl.log["still"]["home"],
              gl.wifi["sleep_home"]))
    both(30 * 60)
    check("SLEEP: 30 min at home asleep writes no anchor point",
          beats_now() == 0 and gl.wifi["state"] == "sleep",
          "beats=%d" % beats_now())
    WLAN.scan_aps = [("home", -90)]                 # home AP out of range
    for i in range(1, 41):
        sv_fix(i * 5.5, 0, 20.0)
        gl.wifi_tick()
    check("SLEEP: leaving ends the home stay while asleep",
          not sv_active() and not gl.at_home())
    # somewhere else, no network there: the first anchor waits for the round
    WLAN.scan_aps, WLAN.ok = [], set()
    gl.wifi["last_check"] = _clock[0]           # (the hour is not over yet)
    n_conn = len(WLAN.connects)
    entered = None
    for k in range(400):
        sv_fix(600 + rnd.uniform(-10, 10), rnd.uniform(-10, 10))
        gl.wifi_tick()
        if entered is None and sv_active():
            entered = k
        if entered is not None and k == entered + 30:
            early = beats_now()
    check("SLEEP: a new stay away wakes the WiFi (nothing found)",
          entered is not None and len(WLAN.connects) > n_conn
          and gl.wifi["state"] == "not found" and not gl.log["still"]["home"])
    check("SLEEP: ... and its first anchor point waits for the round",
          entered is not None and early == 0 and beats_now() >= 1,
          "early=%s beats=%d" % (early if entered is not None else None,
                                 beats_now()))

    # VETO: a departure from a home stay while the home WiFi is in range is
    # a GPS jump (measured: a 194 m multipath jump ended a home stay)
    def jump_away(n, start=160.0, step=3.0):
        for i in range(n):
            sv_fix(start + i * step, 0)
            gl.wifi_tick()

    sv_reset()
    w, calls, out = rv_start_wifi(rv_env, [("home", -40)], ["home"])
    both(150)
    check("VETO: setup - home stay, radio asleep",
          sv_active() and gl.log["still"]["home"]
          and gl.wifi["state"] == "sleep")
    n_before = len(sv_lines())
    jump_away(12)
    both(30)
    check("VETO: a receding jump with home in range keeps the stay",
          sv_active() and len(sv_lines()) == n_before
          and gl.log["still"].get("veto_at") is not None
          and not wl().active(),
          "active=%s new=%d radio=%s" % (sv_active(), len(sv_lines()) - n_before,
                                         wl().active()))
    WLAN.scan_aps = [("home", -88)]
    _clock[0] += gl.STILL_HOME_HOLD
    for i in range(1, 41):
        sv_fix(i * 5.5, 0, 20.0)
        gl.wifi_tick()
    check("VETO: with home out of range (-88 dBm) the departure goes",
          not sv_active() and len(sv_lines()) > n_before)
    sv_reset()
    w, calls, out = rv_start_wifi(rv_env, [], [])  # nothing in range: away
    both(150)
    check("VETO: setup - away stay", sv_active()
          and not gl.log["still"]["home"])
    WLAN.scan_aps, WLAN.ok = [("home", -40)], {"home"}
    for i in range(1, 41):
        sv_fix(i * 5.5, 0, 20.0)
    check("VETO: an away stay is never vetoed",
          not sv_active() and gl.log["still"].get("veto_at") is None)
    sv_reset()
    gl.log["still"]["home"] = True
    check("VETO: no WLAN (tests, no .env) -> no veto",
          gl.home_in_range() is None)
    # connected home with an unreadable RSSI: a scan decides, not a veto
    sv_reset()
    w, calls, out = rv_start_wifi(rv_env, [("home", -40)], ["home"])
    real_status = WLAN.status

    def no_rssi(self, arg=None):
        if arg == "rssi":
            raise ValueError("rssi")
        return real_status(self, arg)
    WLAN.status = no_rssi
    try:
        WLAN.scan_aps = [("home", -90)]
        weak = gl.home_in_range()
        WLAN.scan_aps = [("home", -40)]
        strong = gl.home_in_range()
    finally:
        WLAN.status = real_status
    check("VETO: unknown RSSI while connected -> a scan decides",
          gl.wifi["state"] == "connected" and weak is False and strong is True,
          "weak=%s strong=%s" % (weak, strong))

    # PB_KEEPALIVE: with the radio asleep, one pulse every PB_KEEPALIVE ms
    # keeps a power bank from switching off; none while the radio is on
    # anyway. PB_MODE: "radio" (on/off, default), "scan", "gpio" (a pin)
    scans = [0]
    ons = [0]
    real_scan = WLAN.scan
    real_active = WLAN.active

    def counting_scan(self):
        scans[0] += 1
        return real_scan(self)

    def counting_active(self, x=None):
        if x:
            ons[0] += 1
        return real_active(self, x)
    WLAN.scan = counting_scan
    WLAN.active = counting_active
    check("PB_KEEPALIVE: the default pulse is a scan (radio was too little)",
          gl.PB_MODE == "scan")
    saved_mode, saved_pin = gl.PB_MODE, gl.PB_PIN
    try:
        sv_reset()
        w, calls, out = rv_start_wifi(rv_env, [("home", -40)], ["home"])
        for _ in range(3):
            _clock[0] += gl.PB_KEEPALIVE
            gl.keepalive_tick(_clock[0])
        check("PB_KEEPALIVE: no keep-alive pulse while connected",
              gl.wifi["state"] == "connected" and gl.wifi["pb_scans"] == 0)
        gl._wifi_off(wl(), "sleep")
        gl.PB_MODE = "radio"
        n0, o0 = scans[0], ons[0]
        for _ in range(4):
            _clock[0] += gl.PB_KEEPALIVE // 2
            gl.keepalive_tick(_clock[0])
        check("PB_KEEPALIVE radio: asleep -> radio on/off per PB_KEEPALIVE, no scan",
              scans[0] - n0 == 0 and ons[0] - o0 == 2 and gl.wifi["pb_scans"] == 2
              and not wl().active() and gl.wifi["state"] == "sleep",
              "scans=%d ons=%d" % (scans[0] - n0, ons[0] - o0))
        gl.PB_MODE = "scan"
        n0 = scans[0]
        for _ in range(4):
            _clock[0] += gl.PB_KEEPALIVE // 2
            gl.keepalive_tick(_clock[0])
        check("PB_KEEPALIVE scan: one scan per PB_KEEPALIVE, radio off after",
              scans[0] - n0 == 2 and gl.wifi["pb_scans"] == 4
              and not wl().active() and gl.wifi["state"] == "sleep",
              "scans=%d" % (scans[0] - n0))
        gl.PB_MODE = "gpio"
        gl.PB_PIN = 26
        gl._pb_pin = None
        del _pin_levels[:]
        n0, o0 = scans[0], ons[0]
        _clock[0] += gl.PB_KEEPALIVE
        gl.keepalive_tick(_clock[0])
        lv = list(_pin_levels)
        check("PB_KEEPALIVE gpio: the pin goes high for PB_PULSE_MS, radio untouched",
              [(n, v) for n, v, t in lv] == [(26, 0), (26, 1), (26, 0)]
              and lv[2][2] - lv[1][2] == gl.PB_PULSE_MS
              and scans[0] == n0 and ons[0] == o0 and gl.wifi["pb_scans"] == 5,
              "levels=%r" % lv)
        gl.PB_PIN = 99
        gl._pb_pin = None
        Pin.fail = (99,)
        o0 = ons[0]
        try:
            _clock[0] += gl.PB_KEEPALIVE
            gl.keepalive_tick(_clock[0])
        finally:
            Pin.fail = ()
        check("PB_KEEPALIVE gpio: an unusable pin falls back to scan for good",
              gl.PB_MODE == "scan" and ons[0] - o0 == 1
              and gl.wifi["pb_scans"] == 6, "mode=%s" % gl.PB_MODE)
        gl.PB_MODE = "radio"
        gl._keepalive_env({"KEEPALIVE_MODE": "Scan", "KEEPALIVE_PIN": "12"})
        m1, p1 = gl.PB_MODE, gl.PB_PIN
        gl._keepalive_env({"KEEPALIVE_MODE": "laser", "KEEPALIVE_PIN": "x"})
        check("PB_KEEPALIVE: .env KEEPALIVE_MODE / KEEPALIVE_PIN, bad ones kept",
              m1 == "scan" and p1 == 12 and gl.PB_MODE == "scan"
              and gl.PB_PIN == 12, "%s %s %s %s" % (m1, p1, gl.PB_MODE, gl.PB_PIN))
        saved = gl.PB_KEEPALIVE
        try:
            gl._keepalive_env({"KEEPALIVE": "0"})
            off = gl.PB_KEEPALIVE
            gl._keepalive_env({"KEEPALIVE": "30  # seconds"})
            thirty = gl.PB_KEEPALIVE
            gl._keepalive_env({"KEEPALIVE": "2"})
            kept = gl.PB_KEEPALIVE
            gl._keepalive_env({"KEEPALIVE": "x"})
            kept2 = gl.PB_KEEPALIVE
            gl.PB_KEEPALIVE = 0
            n0 = scans[0]
            _clock[0] += 10 * saved
            gl.keepalive_tick(_clock[0])
            none = scans[0] - n0
        finally:
            gl.PB_KEEPALIVE = saved
        check("PB_KEEPALIVE: .env 0 = off, 30 = 30 s, out of range / invalid kept",
              off == 0 and thirty == 30000 and kept == 30000
              and kept2 == 30000 and none == 0,
              "%s %s %s %s %s" % (off, thirty, kept, kept2, none))
    finally:
        WLAN.scan = real_scan
        WLAN.active = real_active
        gl.PB_MODE, gl.PB_PIN, gl._pb_pin = saved_mode, saved_pin, None

    # BOM: a .env saved by Windows Notepad
    bom = os.path.join(rv_dir, "env_bom")
    with open(bom, "wb") as fh:
        fh.write(b"\xef\xbb\xbfSSID=home\nWPA2=x\nGEO_URL=https://d\nAPI_KEY=k\n")
    check("BOM: a UTF-8 BOM before the first key is ignored",
          "SSID" in gl.load_env(bom) and len(gl._load_nets(gl.load_env(bom))) == 1)

    # REVIEW2-2: a timestamp taken just after now is age 0, not "very old"
    now = _clock[0]
    check("REVIEW2-2: small negative difference is age 0",
          gl._age(now, now + 500) == 0
          and gl._age(now, now + gl.TICKS_SKEW + 1) == gl.TICKS_OLD)
    gl.st.update(valid=True, date="2026-10-17", time="12:00:00",
                 lat=47.5, lon=19.0, pos_at=now + 200)
    check("REVIEW2-2: a fix stamped just after now is still a fix",
          bool(gl.has_fix(now)))
finally:
    shutil.rmtree(rv_dir, ignore_errors=True)

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

# --- tools/trk2gpx.py: .trk -> GPX 1.1 ------------------------------------
import xml.etree.ElementTree as ET
sys.path.insert(0, os.path.join(ROOT, "tools"))
import trk2gpx  # noqa: E402

conv_lines = [
    "#TRK1\n",
    "52.500000,13.400000,10.0,1.2,20261005T120005Z\n",
    "52.500000,13.400000,,,20261005T120006Z\n",
    "-\n",
    "-33.800000,-70.600000,5.5,,20261005T120010Z\n",
    "52.500000,13.400000,10.0,1.0,2026100",   # cut off by a power loss
]
conv_gpx = trk2gpx.convert(conv_lines, "20261005_120005")
try:
    conv_root = ET.fromstring(conv_gpx)
    conv_ok = True
except ET.ParseError as e:
    conv_root, conv_ok = None, False
    conv_gpx += " parse error %s" % e
check("trk2gpx: output is well-formed GPX", conv_ok, conv_gpx[:200])
if conv_root is not None:
    ns = "{http://www.topografix.com/GPX/1/1}"
    segs = conv_root.findall(".//%strkseg" % ns)
    pts = conv_root.findall(".//%strkpt" % ns)
    check("trk2gpx: '-' starts a second trkseg, cut-off tail is dropped",
          len(segs) == 2 and len(pts) == 3,
          "segs=%s pts=%s" % (len(segs), len(pts)))
    first = pts[0]
    check("trk2gpx: ele, hdop and ISO time are carried over",
          first.find(ns + "ele").text == "10.0"
          and first.find(ns + "hdop").text == "1.2"
          and first.find(ns + "time").text == "2026-10-05T12:00:05Z")
    check("trk2gpx: empty ele/hdop produce no tags",
          pts[1].find(ns + "ele") is None and pts[1].find(ns + "hdop") is None)
    check("trk2gpx: negative coordinates kept",
          abs(float(pts[2].get("lat")) + 33.8) < 1e-9
          and abs(float(pts[2].get("lon")) + 70.6) < 1e-9)

# REVIEW-5: GPX 1.1 element order, number validation, escaped name
order_gpx = trk2gpx.convert([
    "52.500000,13.400000,10.0,1.2,20261005T120005Z\n",
    "52.500100,13.400100,<b>,nan,20261005T120006Z\n",
], "a&b")
try:
    order_root = ET.fromstring(order_gpx)
except ET.ParseError as e:
    order_root = None
    order_gpx += " parse error %s" % e
check("REVIEW-5: bad ele/hdop and '&' in the name still give valid XML",
      order_root is not None, order_gpx[:300])
if order_root is not None:
    ns = "{http://www.topografix.com/GPX/1/1}"
    opts = order_root.findall(".//%strkpt" % ns)
    check("REVIEW-5: trkpt children follow GPX 1.1 order ele, time, hdop",
          [c.tag[len(ns):] for c in opts[0]] == ["ele", "time", "hdop"],
          repr([c.tag for c in opts[0]]))
    check("REVIEW-5: non-numeric ele and nan hdop are dropped",
          [c.tag[len(ns):] for c in opts[1]] == ["time"])
    check("REVIEW-5: track name is escaped",
          order_root.find(".//%sname" % ns).text == "a&b")

# DIAG: persistent diagnostic log (what happened on a walk)
def diag_lines(path=None):
    try:
        with open(path or gl.DIAG_FILE) as f:
            return f.read().splitlines()
    except OSError:
        return []


def diag_clear():
    for p in (gl.DIAG_FILE, gl.DIAG_OLD):
        try:
            os.remove(p)
        except OSError:
            pass


saved_diag = (gl.DIAG, gl.DIAG_MAX)
try:
    vals = []
    for v in ("0", "30", "5", "x", "3600"):
        gl.DIAG = 60000
        gl._diag_env({"DIAG": v})
        vals.append(gl.DIAG)
    gl.DIAG = 60000
    gl._diag_env({})
    vals.append(gl.DIAG)
    check("DIAG: .env DIAG 0 off, 10-3600 s, anything else the default",
          vals == [0, 30000, 60000, 60000, 3600000, 60000], repr(vals))

    diag_clear()
    gl.DIAG = 60000
    gl.reset()
    gl.log_reset()
    gl.up_reset()
    _reset_cause[0] = 6
    gl._diag_boot()
    _reset_cause[0] = 1
    gl.st.update(ok=120, err=3, gps_boots=2, loops=1, used=7,
                 view={"GP": 9, "GL": 4}, hdop=1.4)
    gl._diag(125000)
    lines = diag_lines()
    check("DIAG: boot line with the reset cause, then a status line",
          len(lines) == 2 and lines[0].startswith("boot cause 6 ")
          and lines[1].startswith("up 125 utc ")
          and "nmea 120/3 gpsboots 2 loops 1 sats 13/7 " in lines[1]
          and " hdop 1.4 " in lines[1] and " wifi " in lines[1],
          repr(lines))
    gl.st.update(date="2026-10-28", time="11:40:05")
    gl._diag(126000)
    check("DIAG: the GPS UTC time is in the line once known",
          " utc 20261028T114005Z " in diag_lines()[-1], diag_lines()[-1:])

    diag_clear()
    gl.DIAG_MAX = 300
    for k in range(12):
        gl._diag(k * 60000)
    first = diag_lines(gl.DIAG_OLD)
    cur = diag_lines()
    for k in range(12, 24):
        gl._diag(k * 60000)
    second = diag_lines(gl.DIAG_OLD)
    check("DIAG: past DIAG_MAX the file becomes DIAG_OLD (old one replaced)",
          first and cur and os.path.getsize(gl.DIAG_FILE) < 300 + 200
          and second and second[0] != first[0],
          "%d %d %d" % (len(first), len(cur), len(second)))
    gl.DIAG_MAX = saved_diag[1]

    diag_clear()
    gl.DIAG = 0
    gl._diag_boot()
    gl._diag(1000)
    check("DIAG: DIAG=0 writes nothing", diag_lines() == [])
    gl.DIAG = 60000

    saved_file = gl.DIAG_FILE
    gl.DIAG_FILE = os.path.join(diag_dir, "no", "such", "dir", "d.log")
    try:
        gl._diag(1000)
        raised = None
    except Exception as e:
        raised = e
    finally:
        gl.DIAG_FILE = saved_file
    check("DIAG: a write error is printed, never raised", raised is None,
          repr(raised))

    # run(): boot line before the GPS setup, crash line before the reset
    diag_clear()
    _resets[:] = []
    _timers[:] = []
    gl._wdt = None
    write_env(diag_dir, "")

    def diag_boom():
        raise ValueError("diag boom")

    orig_loop = gl._loop
    gl._loop = diag_boom
    try:
        gl.run()
    except SystemExit:
        pass
    finally:
        gl._loop = orig_loop
    lines = diag_lines()
    check("DIAG: run() logs the boot and a crash before machine.reset()",
          len(lines) == 2 and lines[0].startswith("boot cause ")
          and lines[1].startswith("crash up ") and "diag boom" in lines[1]
          and bool(_resets), repr(lines))
    # previous run: read from the last diag lines at boot
    check("UPTIME: seconds read as s, min, h, d",
          [gl._hms(x) for x in (None, 45, 754, 3494, 11040, 183600)]
          == ["-", "45 s", "12 min 34 s", "58 min 14 s", "3 h 04 min",
              "2 d 03 h"],
          repr([gl._hms(x) for x in (45, 754, 3494, 11040, 183600)]))

    def prev_after(text, old=None):
        diag_clear()
        if text is not None:
            with open(gl.DIAG_FILE, "w") as f:
                f.write(text)
        if old is not None:
            with open(gl.DIAG_OLD, "w") as f:
                f.write(old)
        gl._diag_boot()
        return dict(gl.prev_run), diag_lines()[-1], gl._prev_text()

    st_lines = ("boot cause 1 prev_up - mhz 160 mem 1\n"
                "up 63 utc 20261028T115937Z nmea 752/0 x\n"
                "up 3494 utc 20261028T125942Z nmea 9000/0 x\n")
    p1, boot1, t1 = prev_after(st_lines)
    check("UPTIME: the last status line gives the previous run's uptime",
          p1 == {"up": 3494, "end": "2026-10-28 12:59:42Z", "how": "status"}
          and boot1.startswith("boot cause 1 prev_up 3494 ")
          and t1 == "58 min 14 s (last line 2026-10-28 12:59:42Z)",
          "%r %r %r" % (p1, boot1, t1))
    p2, _, t2 = prev_after(st_lines + "crash up 3500 ValueError('x')\n")
    p3, boot3, t3 = prev_after(st_lines + "boot cause 1 prev_up 3494 mhz 1\n")
    p4, _, _ = prev_after("up 125 utc - nmea 1/0\n")
    p5, _, _ = prev_after(None, old=st_lines)
    p6, boot6, t6 = prev_after(None)
    check("UPTIME: crash, a run without a status line, no GPS time",
          p2["up"] == 3500 and p2["how"] == "crash"
          and t2 == "58 min 20 s, ended by a crash"
          and p3 == {"up": 0, "end": None, "how": "short"}
          and t3 == "under 60 s" and " prev_up 0 " in boot3
          and p4 == {"up": 125, "end": None, "how": "status"},
          "%r %r %r %r" % (p2, t2, p3, p4))
    check("UPTIME: just rotated -> diag.1; no log at all -> unknown",
          p5["up"] == 3494 and p6["up"] is None and t6 is None
          and " prev_up - " in boot6, "%r %r %r" % (p5, p6, boot6))
    prev_after(st_lines)
    rows = {r[2]: r[3] for r in gl._web_rows()}
    page = gl._web_html()
    check("UPTIME: page shows readable times, the JSON keeps seconds",
          isinstance(rows["uptime"], int) and rows["prev_uptime"] == 3494
          and "<td>Previous run</td><td>58 min 14 s (last line "
              "2026-10-28 12:59:42Z)</td>" in page
          and "<td>Uptime</td><td>" in page and "Uptime (s)" not in page,
          page[page.find("Uptime"):page.find("Uptime") + 200])

    import inspect
    loop_src = inspect.getsource(gl._loop)
    check("DIAG: _loop writes a line every DIAG ms",
          "_diag(now)" in loop_src and "DIAG" in loop_src)
finally:
    gl.DIAG, gl.DIAG_MAX = saved_diag

print()
print("%d passed, %d failed" % (passed, failed))
if failures:
    print("failures:")
    for f in failures:
        print(" ", f)
sys.exit(1 if failed else 0)
