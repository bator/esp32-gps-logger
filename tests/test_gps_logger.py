#!/usr/bin/env python3
"""CPython harness for gps_logger upload / crash / WiFi logic. Not deployed."""
from __future__ import print_function

import builtins
import gc as real_gc
import hashlib
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
_uart_sv = [None]                       # NAV-SVINFO answer to a poll
_uart_sat = [None]                      # NAV-SAT answer to a poll (M10)
_uart_ver = [None]                      # MON-VER answer to a poll
_uart_tx = []
_uart_rx = bytearray()
_uart_ack = [True]
_uart_drop = [0]
_uart_nak = set()
_uart_nak_frames = set()                 # whole frames refused (NAK)
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
        if data[:4] == b"\xb5\x62\x01\x30" and _uart_sv[0]:
            _uart_rx.extend(_uart_sv[0])
        if data[:4] == b"\xb5\x62\x01\x35" and _uart_sat[0]:
            _uart_rx.extend(_uart_sat[0])
        if data[:4] == b"\xb5\x62\x0a\x04" and _uart_ver[0]:
            _uart_rx.extend(_uart_ver[0])
        # a u-blox acknowledges a UBX-CFG frame (unless told to drop it)
        if _uart_ack[0] and data[:3] == b"\xb5\x62\x06":
            if _uart_drop[0]:
                _uart_drop[0] -= 1
            elif (data[2], data[3]) in _uart_silent:
                pass
            else:
                kind = (0x00 if (data[2], data[3]) in _uart_nak
                        or data in _uart_nak_frames else 0x01)
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
    dt = (2000, 1, 1, 5, 0, 0, 0, 0)     # the clock at boot (no RTC battery)
    sets = []

    def memory(self, data=None):
        if data is None:
            return RTC.mem
        RTC.mem = data if isinstance(data, bytes) else bytes(data)
        return None

    def datetime(self, t=None):
        # v1.29: (year, month, day, weekday, hours, minutes, seconds,
        # subseconds); weekday 0 = Monday
        if t is None:
            return RTC.dt
        RTC.dt = tuple(t)
        RTC.sets.append(tuple(t))
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


class _TlsCtx(object):
    """MicroPython's tls.SSLContext: up_reset() makes one at boot and
    upload_tick checks it before a POST (CPython's ssl refuses CERT_NONE
    while check_hostname is on)."""
    made = []

    def __init__(self, proto):
        self.verify_mode, self.ca = None, None
        _TlsCtx.made.append(self)

    def load_verify_locations(self, cadata=None):
        self.ca = cadata

    def wrap_socket(self, s, server_hostname=None):
        return s


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

    tls = types.ModuleType("tls")
    tls.PROTOCOL_TLS_CLIENT, tls.CERT_NONE, tls.CERT_REQUIRED = 2, 0, 2
    tls.SSLContext = _TlsCtx
    sys.modules["tls"] = tls

    fake_gc = types.ModuleType("gc")
    fake_gc.threshold = lambda *a, **k: None
    fake_gc.collect = lambda: real_gc.collect()
    fake_gc.mem_free = lambda: 123456
    fake_gc.mem_alloc = lambda: 65432
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
    m.LOWIDF_MARK = os.path.join(diag_dir, "lowidf")
    m.HIST_DIR = os.path.join(diag_dir, "hist")         # (day history)
    m.HOME_APS_FILE = None               # tests that keep BSSIDs set a path
    m.WEB = False                        # radio-sleep tests; WEB has its own
    m.HTTP_POST = lambda url, data, headers, timeout: _post(
        url, data=data, headers=headers, timeout=timeout)
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

# OLED-FLASH: the free flash on the screen comes through _flash_kb's 30 s
# cache; draw() runs every REFRESH (500 ms) and statvfs walks the LittleFS
_sv_calls = []
_real_statvfs = os.statvfs
os.statvfs = lambda p: _sv_calls.append(p) or _real_statvfs("/")
gl._fl[:] = [None, None]
gl.oled.ops[:] = []
try:
    for _ in range(6):
        gl.draw()
        _clock[0] += gl.REFRESH
finally:
    os.statvfs = _real_statvfs
_flash_txt = [op[1][0] for op in gl.oled.ops
              if op[0] == "text" and str(op[1][0]).startswith("Flash ")]
check("OLED-FLASH: draw() every REFRESH reads statvfs once per 30 s, in kB",
      len(_sv_calls) == 1 and len(_flash_txt) == 6
      and _flash_txt[0] == "Flash %d kB" % gl._fl[0],
      "%r %r" % (_sv_calls, _flash_txt[:2]))

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
            _clock[0] += gl.CONNECT_TIMEOUT
            gl.wifi_tick()

    def start_wifi(env_text, aps, ok):
        reset_wlan()
        WLAN.scan_aps = aps
        WLAN.ok = set(ok)
        write_env(tmpdir, env_text)
        gl.wifi_reset()
        gl.up_reset()
        gl.wifi_start()
        drain_wifi()
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
    # WR-a: a good scan that lists none of them: the boot round tries every
    # configured network (12 s each, radio on), later rounds only every
    # UNLISTED_EVERY rounds; a hidden one (never listed by name) every round
    def wr_rounds(env_text, n):
        start_wifi(env_text, [("unrelated", -40)], [])
        tries = []
        for _ in range(n):
            WLAN.connects = []
            gl.wifi["state"] = "not found"
            gl.wifi["last_check"] = _clock[0] - gl.wifi["backoff"]
            gl.wifi_tick()
            drain_wifi()
            tries.append([c[0] for c in WLAN.connects])
        return tries
    every = getattr(gl, "UNLISTED_EVERY", 4)
    tries = wr_rounds(triple, every)
    hid = wr_rounds(env_hid_home, 2)
    check("WR-a: unlisted networks are tried at boot and every Nth round",
          tries == [[]] * (every - 1) + [["home", "phone", "cabin"]]
          and hid == [["home"], ["home"]], "%r %r" % (tries, hid))
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
    rl_name = "_rd_line" if hasattr(gl, "_rd_line") else "_readline"
    orig_rl = getattr(gl, rl_name)

    def boom_line(f):
        memerr.append(1)
        raise MemoryError("heap")

    setattr(gl, rl_name, boom_line)
    try:
        gl.up_reset()
        check("B2: count_points/up_reset catch MemoryError (no reset)",
              True)
    except MemoryError:
        check("B2: count_points/up_reset catch MemoryError (no reset)", False)
    finally:
        setattr(gl, rl_name, orig_rl)
        gl.up["url"] = "https://dawarich.example.com"
        gl.up["key"] = "secret-key"
        gl.wifi["state"] = "connected"

    setattr(gl, rl_name, boom_line)
    try:
        pump()
        check("B2: upload_tick MemoryError skips the file (no infinite loop)",
              gl.up["skip"] == path and gl.up["state"] == "low mem")
    except MemoryError:
        check("B2: upload_tick MemoryError skips the file (no infinite loop)",
              False, "MemoryError escaped")
    finally:
        setattr(gl, rl_name, orig_rl)

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

    # TRK-END: the footer ends a .trk. A failed longer flush, then a shorter
    # good one, leaves old point lines after "#END": dropped drift points
    # that must not be uploaded; the file still counts as complete
    end_dir = os.path.join(tmpdir, "trkend")
    os.makedirs(end_dir)
    gl.LOG_DIR = end_dir
    te = os.path.join(end_dir, "20261005_130000.trk")
    good = [trk_line(52.5 + i * 0.001, 13.4, "2026-10-05T13:00:%02dZ" % i)
            for i in range(3)]
    drift = [trk_line(53.9, 14.9, "2026-10-05T13:00:%02dZ" % (i + 10))
             for i in range(4)]
    with open(te, "wb") as f:
        f.write((gl.HEADER + "".join(good) + gl.FOOTER + "".join(drift)
                 + "52.6,13.5,,,2026100").encode())
    n, n_ok, size, new_off, eof = gl.read_batch(te, 0)
    te_left = gl.count_points()
    online(end_dir)
    with open(te, "wb") as f:
        f.write((gl.HEADER + "".join(good) + gl.FOOTER + "".join(drift)
                 + gl.FOOTER).encode())
    pump()
    te_locs = locations()
    check("TRK-END: reading stops at #END; what follows is not uploaded",
          (n, n_ok, eof) == (3, 3, True) and te_left == 3
          and len(te_locs) == 3 and not os.path.exists(te)
          and all(loc["geometry"]["coordinates"][1] < 53 for loc in te_locs),
          "n=%s n_ok=%s eof=%s left=%s posted=%d exists=%s" % (
              n, n_ok, eof, te_left, len(te_locs), os.path.exists(te)))

    # RB-INTO: a .trk is read with readinto into the one preallocated line
    # buffer, not f.readline(LINE_MAX) (a new LINE_MAX bytes per line)
    rb_p = os.path.join(end_dir, "20261005_140000.trk")
    make_trk(rb_p, gl.UPLOAD_BATCH + 7)
    io_calls = {"readline": 0, "readinto": 0, "foreign": 0}
    rbuf = getattr(gl, "_rbuf", None)

    class _CountFile(object):
        def __init__(self, f):
            self.f = f

        def __enter__(self):
            return self

        def __exit__(self, *a):
            self.f.close()

        def readline(self, *a):
            io_calls["readline"] += 1
            return self.f.readline(*a)

        def readinto(self, b):
            io_calls["readinto"] += 1
            if rbuf is None or (b is not rbuf
                                and getattr(b, "obj", None) is not rbuf):
                io_calls["foreign"] += 1
            return self.f.readinto(b)

        def __getattr__(self, k):
            return getattr(self.f, k)
    real_open_rb = builtins.open
    builtins.open = lambda name, *a, **k: (
        _CountFile(real_open_rb(name, *a, **k)) if name == rb_p
        else real_open_rb(name, *a, **k))
    try:
        rb_res = gl.read_batch(rb_p, 0)
        rb_res2 = gl.read_batch(rb_p, rb_res[3])
    finally:
        builtins.open = real_open_rb
    check("RB-INTO: read_batch reads into the preallocated buffer, no readline",
          io_calls["readline"] == 0 and io_calls["readinto"] > 0
          and io_calls["foreign"] == 0
          and rb_res[:2] == (gl.UPLOAD_BATCH, gl.UPLOAD_BATCH)
          and rb_res2[:2] == (7, 7) and rb_res2[4], "%r %r %r" % (
              io_calls, rb_res, rb_res2))

    # RB-SAME: the .trk JSON is byte for byte what the old per-point
    # "%.6f" / "%.1f" formatting gave (canonical bytes are copied as they
    # are, anything else is formatted as before)
    rows = [("52.512345", "13.400001", "10.0", "1.2", "20261005T120005Z"),
            ("-33.800000", "-70.600000", "", "", "20261005T120006Z"),
            ("47.5", "19", "123", "0.9", "20261005T120007Z"),
            ("-0.000000", "0.000000", "-0.0", "", "20261005T120008Z"),
            ("+47.100000", " 19.200000", "1e2", "inf", "20261005T120009Z"),
            ("47.1234567", "019.123456", "8848.86", "2", "20261005T120010Z"),
            ("90.000000", "-180.000000", "nan", "0.0", "20261005T120011Z"),
            ("47.0", "19.0", "abc", "x", "20261005T120012Z")]
    same_p = os.path.join(end_dir, "20261005_150000.trk")
    with open(same_p, "wb") as f:
        f.write((gl.HEADER + "".join(",".join(r) + "\n" for r in rows)
                 + gl.FOOTER).encode())
    want = []
    for la, lo, el, hd, tc in rows:
        ef = float(el) if el and el not in ("abc",) else None
        if ef is not None and ef != ef or ef in (float("inf"),):
            ef = None
        hf = float(hd) if hd and hd != "x" else None
        extra = ""
        if ef is not None:
            extra += '"altitude":%.1f,' % ef
        if hf is not None and hf == hf and abs(hf) != float("inf"):
            extra += '"horizontal_accuracy":%.1f,' % (hf * gl.HDOP_M)
        iso = "%s-%s-%sT%s:%s:%sZ" % (tc[0:4], tc[4:6], tc[6:8], tc[9:11],
                                      tc[11:13], tc[13:15])
        want.append('{"type":"Feature","geometry":{"type":"Point",'
                    '"coordinates":[%.6f,%.6f]},"properties":{'
                    '"timestamp":"%s",%s"device_id":"%s"}}'
                    % (float(lo), float(la), iso, extra, gl.DEVICE_ID))
    want_b = ('{"locations":[' + ",".join(want) + "]}").encode()
    n, n_ok, size, new_off, eof = gl.read_batch(same_p, 0)
    got_b = bytes(gl._body[:size])
    check("RB-SAME: .trk Overland JSON byte-identical to the old formatting",
          (n, n_ok, eof) == (len(rows), len(rows), True) and got_b == want_b,
          "%r\n%r" % (got_b[:300], want_b[:300]))

    # RB-LONG: a point line of exactly LINE_MAX bytes is read, one byte
    # longer is skipped whole, the points around it are kept
    lm_p = os.path.join(end_dir, "20261005_160000.trk")
    pt = trk_line(52.5, 13.4, "2026-10-05T16:00:00Z", hdop="1.0")
    fit = " " * (gl.LINE_MAX - len(pt)) + pt
    over = " " * (gl.LINE_MAX + 1 - len(pt)) + pt
    with open(lm_p, "wb") as f:
        f.write((gl.HEADER + pt + fit + over + "x" * 3000 + "\n" + pt
                 + gl.FOOTER).encode())
    n, n_ok, size, new_off, eof = gl.read_batch(lm_p, 0)
    check("RB-LONG: LINE_MAX line read, longer lines skipped, rest kept",
          len(fit) == gl.LINE_MAX and (n, n_ok, eof) == (3, 3, True)
          and new_off == os.path.getsize(lm_p), "n=%s n_ok=%s eof=%s %s" % (
              n, n_ok, eof, new_off))

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
    gl.wifi_start()
    drain_wifi()
    check("K6: empty scan falls back through configured networks",
          [c[0] for c in WLAN.connects] == ["home", "PhoneHotspot"],
          "connects=%s" % (WLAN.connects,))
    check("K6: hidden net connects when not in the scan",
          gl.wifi["state"] == "connected" and gl.wifi["ssid"] == "PhoneHotspot")

    # --- K7: a leftover fail reason does not fail a good net at once --------
    reset_wlan()
    WLAN.scan_aps = [("home", -30), ("PhoneHotspot", -50)]
    WLAN.ok = set(["PhoneHotspot"])
    WLAN.stale_fail = 2
    WLAN.leftover = 4  # STAT_WRONG_PASSWORD leftover
    write_env(tmpdir, env_hot)
    gl.wifi_reset()
    gl.up_reset()
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

    # WR-31: five neighbours' hidden APs taking turns as the strongest one:
    # each hidden-home try drops the hotspot for >= 12 s. Failed hidden
    # roams grow their own hold (10, 20, 40, 60 min); the visible-home
    # check stays at ROAM_CHECK
    w = start_wifi(env_hid_home, [("phone", -40)], ["phone"])
    WLAN.ok = set(["phone"])
    gl.st["speed"] = None
    WLAN.connects = []
    for i in range(24):                  # 2 h of ROAM_CHECKs
        _clock[0] += gl.ROAM_CHECK
        WLAN.scan_aps = [("", -50, bytes([0x20 + i % 5]) * 6),
                         ("phone", -40)]
        gl.wifi["roam_at"] = _clock[0] - gl.ROAM_CHECK
        if WLAN.instances:
            WLAN.instances[-1]._status = 3
        gl.wifi["state"] = "connected"
        gl.wifi["home"] = False
        gl.wifi["ssid"] = "phone"
        gl.wifi_tick()
        drain_wifi()
    hid_tries = len([c for c in WLAN.connects if c[0] == "home"])
    check("WR-31: failed hidden-home roams back off (not every 5 min)",
          1 <= hid_tries <= 6
          and (gl.wifi.get("roam_backoff") or 0) == gl.ROAM_CHECK,
          "tries=%d backoff=%s" % (hid_tries, gl.wifi.get("roam_backoff")))

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
                  wfail=True, online=False)
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

    # BOOT-DOC: README, the diag docstring and the page say what main.py /
    # run() do and what machine.reset_cause() means on MicroPython 1.29
    # (ports/esp32/modmachine.c: 1 PWRON incl. brownout, 2 HARD incl.
    # panic and machine.reset(), 3 WDT, 4 DEEPSLEEP, 5 SOFT; there is no 6)
    with open(os.path.join(ROOT, "README.md")) as f:
        readme = f.read()
    with open(os.path.join(ROOT, "web.html")) as f:
        page_boot = f.read()
    guard = readme[readme.index("**Boot loop guard:**"):]
    guard = " ".join(guard[:guard.index("\n\n**")].split())
    import inspect
    run_src = inspect.getsource(gl.run)
    check("BOOT-DOC: boot loop guard text matches main.py and run()",
          "time.sleep(1)" in run_src and "_boot_fail_bump()" in run_src
          and boot_ns["BOOT_FAIL_MAX"] == 5 and "at 5" in guard
          and [boot_ns["_boot_fail_delay"](n) for n in (1, 2, 7)]
          == [5, 10, 30] and "5 s, 10 s" in guard and "30 s" in guard
          and "after 1 s (no growing delay)" in guard
          and "counted at the next boot, no delay" in guard
          and gl.BOOT_OK_MS == 60000 and "60 s (`BOOT_OK_MS`)" in guard
          and "panic" in guard, guard[:300])
    check("BOOT-DOC: boot causes as MicroPython 1.29 numbers them (no 6)",
          "6 crash" not in readme and "6 crash" not in gl._diag_boot.__doc__
          and "2 hard reset" in readme and "5 soft reset" in readme
          and "2 hard reset" in gl._diag_boot.__doc__
          and 'var BOOT = {0: "other reset", 1: "power on", 2: "hard reset", '
              '3: "watchdog", 4: "deep sleep", 5: "soft reset"};' in page_boot)

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
    check("UBX: GSA every GSA_EVERY s, RMC and GGA every fix",
          gl._ubx(6, 1, bytes((0xF0, 2, gl.GPS_HZ * gl.GSA_EVERY))).hex()
          in frames
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
          and len(_uart_tx) == len(gl.GPS_MSGS) + 4   # + the NAV-PVT stop
          and gl.st["setup_ok"] is True)
    _uart_tx[:] = []
    _uart_drop[0] = 2                               # two frames lost
    gl.gps_setup()
    check("ACK: a dropped frame is sent again until acknowledged",
          len(_uart_tx) == len(gl.GPS_MSGS) + 6
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
          and len(_uart_tx) == gl.GPS_ACK_TRIES + len(gl.GPS_MSGS) + 1
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
    def cfg_n():                                    # UBX-CFG frames only
        return len([f for f in _uart_tx if f[2:3] == b"\x06"])

    gl.reset()
    _uart_tx[:] = []
    gl.gps_setup()
    n_frames = cfg_n()
    gl.handle(gsv_line)                             # still in the UART buffer
    _clock[0] += gl.GPS_RETRY
    gl.gps_check(_clock[0])
    check("UBX: sentences queued before the setup do not count",
          cfg_n() == n_frames and gl.st["extra"] == 0)
    gl.handle(gsv_line)
    gl.handle(nmea("GPVTG,,T,,M,0.0,N,0.0,K,A"))
    gl.gps_check(_clock[0])
    check("UBX: GSV / VTG after the setup -> it is sent again",
          cfg_n() == 2 * n_frames and gl.st["extra"] == 0
          and gl.st["setup_wait"] == gl.GPS_RETRY)
    _clock[0] += 1500
    gl.handle(gsv_line)
    _clock[0] += gl.GPS_RETRY
    gl.gps_check(_clock[0])
    check("UBX: one that acknowledges and ignores: again after GPS_RETRY, "
          "then doubling",
          cfg_n() == 3 * n_frames and gl.st.get("ack_n") == 2
          and gl.st["setup_wait"] == 2 * gl.GPS_RETRY,
          "frames=%d wait=%d" % (cfg_n(), gl.st["setup_wait"]))
    _clock[0] += 10 * gl.GPS_RETRY_MAX
    gl.gps_check(_clock[0])
    check("UBX: a receiver that took it is never sent it again",
          cfg_n() == 3 * n_frames)
    # a receiver that never answers: less and less often
    _uart_ack[0] = False
    try:
        gl.reset()
        gl.gps_setup()
        sends = []
        for _ in range(40):
            _clock[0] += 1500
            gl.handle(gsv_line)
            n0 = cfg_n()                 # setup frames (not list polls)
            gl.gps_check(_clock[0])
            if cfg_n() > n0:
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
          and len(_uart_tx) == 2 + gl.GPS_ACK_TRIES + len(gl.GPS_MSGS),
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
    # WR-30: a hidden home behind a strong blank AP gets that RSSI, so its
    # failed first association (the C6 cold-boot 202) is retried at once;
    # R2-c: only behind one of its learned BSSIDs (here from HOME_APS_FILE,
    # as after a reboot): a stranger's blank AP cost 2 x 12 s every round
    saved_haf = gl.HOME_APS_FILE
    gl.HOME_APS_FILE = os.path.join(rv_dir, "home_aps")
    try:
        with open(gl.HOME_APS_FILE, "w") as f:
            f.write("313131313131\n")
        w, calls, out = rv_start_wifi(
            rv_env + "SSID_HIDDEN=1\n",
            [("", -55, b"\x31" * 6), ("phone", -40)],
            ["home", "phone"], fail_first=("home",))
        check("WR-30: a hidden home behind a strong blank AP is retried at "
              "once", calls == ["home", "home"] and w["ssid"] == "home"
              and w["state"] == "connected", "calls=%s" % calls)
        os.remove(gl.HOME_APS_FILE)
        w, calls, out = rv_start_wifi(
            rv_env + "SSID_HIDDEN=1\n",
            [("", -55, b"\x32" * 6), ("phone", -40)],
            ["home", "phone"], fail_first=("home",))
        check("R2-c: behind an unlearned blank AP the hidden home is tried "
              "first but not retried at once", calls == ["home", "phone"]
              and w["ssid"] == "phone", "calls=%s" % calls)
    finally:
        gl.HOME_APS_FILE = saved_haf
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

    def sv_m_ok(line):
        return line.count(",") == 4

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

    # RING: the departure ring is preallocated; a stay's state keeps no
    # per-sample objects (as tuples of line, time and position the 90
    # samples took ~20 kB of the board's heap and broke uploads)
    def deep_size(o, seen=None):
        seen = set() if seen is None else seen
        if id(o) in seen:
            return 0
        seen.add(id(o))
        n = sys.getsizeof(o)
        if isinstance(o, dict):
            n += sum(deep_size(k, seen) + deep_size(v, seen)
                     for k, v in o.items())
        elif isinstance(o, (list, tuple)):
            n += sum(deep_size(v, seen) for v in o)
        return n
    still_size = deep_size(gl.log["still"])
    check("RING: a 30 min stay's state stays small (no per-sample objects)",
          sv_active() and still_size < 4000, "%d bytes" % still_size)

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
    check("RING: the way out rebuilt from the ring is a regular track line",
          bool(new) and len(new[0].split(",")) == 5
          and new[0].split(",")[2:4] == ["100.0", "1.0"]
          and len(new[0].split(",")[4]) == 17 and new[0].endswith("Z\n"),
          repr(new[:2]))
    # RING: the rebuilt way out is the very line _sample_line writes for
    # the fix: an elevation of 123.45 m kept as a single float came back as
    # 123.4 where the line said 123.5 (HDOP 1.25 likewise). (On the board's
    # single-precision floats: MICROPYTHON_F32, tests/mpy_unix_logger.py.)
    def ring_lines(alt, hdop, lat0, lon0, date="2026-10-18"):
        gl.log_reset()
        gl.log["still"] = sl = gl._still_state()
        sl.update(active=True, anchor=(lat0, lon0), aref=(lat0, lon0), n=10)
        want = []
        for k in range(gl.STILL_DEP_MAX + 10):
            gl.st.update(valid=True, date=date, alt=alt, hdop=hdop,
                         hdop_at=_clock[0], fixtype=3, std=None,
                         time="09:%02d:%02d" % (k // 60, k % 60),
                         lat=lat0 + k * 0.0000271828,
                         lon=lon0 + k * 0.0000314159)
            want.append(gl._sample_line(_clock[0], None, None)[1])
            gl._ring_add(sl, _clock[0], (gl.st["lat"], gl.st["lon"]),
                         10.0 + k)
        gl._still_leave()
        return want[-gl.STILL_DEP_MAX:], list(gl.log["batch"])

    want, got = ring_lines(123.45, 1.25, 47.49791234567, 19.04024561234)
    bad = [(w, g) for w, g in zip(want, got) if w != g]
    check("RING: the way out is the line _sample_line writes (123.45 m, "
          "HDOP 1.25)", len(got) == len(want) and not bad, repr(bad[:2]))
    want, got = ring_lines(-0.04, 2.35, 51.47781234567, -0.00213141593)
    bad = [(w, g) for w, g in zip(want, got) if w != g]
    check("RING: ... also across the prime meridian (-0.0 m, HDOP 2.35)",
          len(got) == len(want) and not bad, repr(bad[:2]))
    # times in the ring count from its first sample: from 2088 on seconds
    # since 2020 overflowed its 32-bit array (CPython raised, the board
    # stamped the way out 1951)
    try:
        want, got = ring_lines(100.0, 1.0, 47.5, 19.0, date="2088-01-19")
        err = None
    except OverflowError as e:
        want, got, err = [1], [], e
    check("RING: ring times after 2088 do not overflow", err is None
          and got == want and got[0].endswith("20880119T090010Z\n"),
          "err=%r first=%r" % (err, got[:1]))
    # the way out is the whole ring: _ring_add starts it over at a sample
    # that falls back, so the walk back for the receding run (removed)
    # always reached its start
    frnd = random.Random(23)
    bad = 0
    for _ in range(200):
        gl.log_reset()
        gl.log["still"] = sl = gl._still_state()
        sl.update(active=True, anchor=(47.5, 19.0), aref=(47.5, 19.0), n=10)
        gl.st.update(valid=True, date="2026-10-18", alt=None, hdop=None,
                     fixtype=3)
        dd, ref = frnd.uniform(0, 300), []
        for k in range(frnd.randint(1, 300)):
            dd = max(0.0, dd + frnd.choice((-7, -5.01, -4.99, 0, 2, 6)))
            gl.st["time"] = "%02d:%02d:%02d" % (10 + k // 3600, k // 60 % 60,
                                                k % 60)
            gl._ring_add(sl, _clock[0], (47.5 + k * 1e-4, 19.0), dd)
            ref.append((k, dd))
        ref = ref[-gl.STILL_DEP_MAX:]
        i = len(ref) - 1
        while i > 0 and ref[i - 1][1] <= ref[i][1] + gl.STILL_TOL:
            i -= 1
        gl._still_leave()
        tcs = [l.split(",")[4] for l in gl.log["batch"]]
        got = [int(t[9:11]) * 3600 + int(t[11:13]) * 60 + int(t[13:15])
               - 36000 for t in tcs]
        bad += [k for k, _ in ref[i:]] != got
    check("RING: the way out is the whole ring, a run that kept receding",
          bad == 0, "mismatches=%d" % bad)
    gl.log_reset()

    # walking (1.4 m/s) is never a stay, even with the speed reading low
    sv_reset()
    for i in range(10 * 60):
        sv_fix(i * 1.4, 0, 1.0)
    walk_pts = len(sv_lines())
    check("STAY: a 10 min walk is never stationary, fully recorded",
          sv_active() is False and walk_pts >= 190,
          "points=%d" % walk_pts)
    check("MINMOVE: 1 m default - walking (1.4 m/s) gets ~a point a second",
          gl.MIN_MOVE == 1 and walk_pts >= 550, "points=%d" % walk_pts)
    saved_mm = gl.MIN_MOVE
    try:
        gl.MIN_MOVE = 3
        sv_reset()
        for i in range(10 * 60):
            sv_fix(i * 1.4, 0, 1.0)
        walk3 = len(sv_lines())
    finally:
        gl.MIN_MOVE = saved_mm
    # standing still: the receiver's static hold keeps the fix within cm;
    # the 5-fix mean wobbles well under a metre
    sv_reset()
    sv_fix(0, 0, 0.0)
    before = len(sv_lines())
    for i in range(30):                # (no rnd: later tests use its sequence)
        sv_fix((i * 7 % 7 - 3) * 0.1, (i * 5 % 7 - 3) * 0.1, 0.0)
    still_new = len(sv_lines()) - before
    check("MINMOVE: 3 m kept a point every ~2-3 s; standing adds none",
          walk3 < 0.5 * walk_pts and still_new == 0,
          "3m=%d 1m=%d still=%d" % (walk3, walk_pts, still_new))

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
          and home_file in gl.pending(),
          "active=%s home=%s file=%s batch=%d pending=%r" % (
              sv_active(), gl.log["still"].get("home"), gl.log["file"],
              len(gl.log["batch"]), gl.pending()))
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
    # the clock counts from 2020: a small int on MicroPython (< 2**30)
    gl.st.update(date="2026-10-18", time="08:00:00")
    s20 = gl._fix_s() if hasattr(gl, "_fix_s") else None
    check("HB: internal seconds count from 2020 and stay below 2**30",
          s20 is not None and 0 < s20 < 2 ** 30
          and gl._stamp_s(s20)[0] == "2026-10-18T08:00:00Z"
          and gl._fix_secs() == calendar.timegm((2026, 10, 18, 8, 0, 0, 0, 0, 0)),
          "s20=%s" % s20)
    # a receiver past a GPS week rollover says 1999 (read as 2099) or 2000:
    # such a fix is not used, moving or in a stay. (From 2088 on its
    # seconds overflowed the departure ring: CPython raised, the board
    # stamped the way out 1951; the README's week-rollover note.)
    def hb_dated(date, n, east=0.0):
        errs = []
        for k in range(n):
            _clock[0] += gl.SAMPLE
            gl.st.update(valid=True, date=date, time="08:00:%02d" % k,
                         lat=47.5, lon=19.0 + (east + k * 6.0) * M_LON,
                         alt=100.0, pos_at=_clock[0], hdop=1.0,
                         hdop_at=_clock[0], status="A", quality=1,
                         fixtype=3, speed=22.0)
            try:
                gl.log_tick()
            except Exception as e:
                errs.append(e)
        return errs

    sv_reset()
    errs = hb_dated("2099-08-22", 20) + hb_dated("2000-01-01", 20)
    check("HB: a fix dated 2099 or 2000 (GPS week rollover) is not used",
          not errs and not sv_lines() and gl.log["clock"] is None,
          "errs=%r lines=%s" % (errs[:1], sv_lines()[:1]))
    sv_reset()
    sv_drift(150)
    ok0, n0 = sv_active(), len(sv_lines())
    errs = hb_dated("2099-08-22", 40, east=200.0)
    check("HB: ... nor in a stay (no overflow, no way out stamped wrong)",
          ok0 and not errs and sv_active() and len(sv_lines()) == n0,
          "errs=%r active=%s new=%s" % (errs[:1], sv_active(),
                                        sv_lines()[n0:n0 + 1]))

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

    # STAYWIFI: without a fix on the way (a train), an away stay ended only
    # at home, and every heartbeat until then was stamped at the old place.
    # The keep-alive scans tell that the access points around are gone.
    def ka_setup(aps):
        sv_reset()
        sv_drift(150)
        w = gl.network.WLAN(gl.network.STA_IF)
        gl.wifi.update(wlan=w, state="not found",
                       nets=[{"ssid": "phone", "pw": "x", "home": False}])
        WLAN.scan_aps = aps
        return gl.log["still"]["active"]

    def ka_run(secs):
        for _ in range(secs):
            hb_fix(0, 0, nofix=True)
            gl.keepalive_tick(_clock[0])

    def ka_beats():
        return len([l for l in sv_lines() if l.split(",")[2] == ""])

    office = ("office", -60, b"\x01" * 6)
    cafe = ("cafe", -85, b"\x04" * 6)
    phone = ("phone", -40, b"\x09" * 6)
    train = ("MAVSTART-WIFI", -70, b"\x03" * 6)
    saved_aps, saved_mode = WLAN.scan_aps, gl.PB_MODE
    gl.PB_MODE = "scan"
    try:
        ok_a = ka_setup([office, cafe, phone])
        ka_run(60)
        WLAN.scan_aps = [phone, train]   # the phone hotspot comes along
        ka_run(3 * 60)
        left = not gl.log["still"]["active"]
        b0 = ka_beats()
        ka_run(30 * 60)
        beats_after = ka_beats() - b0
        # the same place: APs fading in and out for 40 s is no departure
        ok_b = ka_setup([office, cafe])
        ka_run(60)
        WLAN.scan_aps = []
        ka_run(40)
        WLAN.scan_aps = [cafe]
        ka_run(10 * 60)
        stayed = gl.log["still"]["active"]
        b0 = ka_beats()
        ka_run(10 * 60)
        stayed_beats = ka_beats() - b0
        # nothing around while learning: no WiFi verdict, as before
        ok_c = ka_setup([])
        ka_run(60)
        WLAN.scan_aps = [train]
        ka_run(5 * 60)
        no_aps = gl.log["still"]["active"]
    finally:
        WLAN.scan_aps, gl.PB_MODE = saved_aps, saved_mode
        gl.wifi.update(wlan=None, state="offline", nets=[])
    check("STAYWIFI: setup - away stays", ok_a and ok_b and ok_c)
    check("STAYWIFI: the stay's access points gone -> the stay ends, no fix",
          left)
    check("STAYWIFI: no heartbeat at the old place after leaving",
          beats_after == 0, "beats=%d" % beats_after)
    check("STAYWIFI: APs fading for 40 s at the same place keep the stay",
          stayed and stayed_beats >= 1, "beats=%d" % stayed_beats)
    check("STAYWIFI: no access points while learning -> the stay goes on",
          no_aps)
    # STAYWIFI: the access points gone write a way out only when the last
    # sample was out past STILL_NEAR; otherwise the ring is the stay's
    # drift (its last receding run, within metres of the anchor) and the
    # stay just ends
    gl.PB_MODE = "scan"
    res = []
    try:
        for last in ((1, 3, 6, 10), (60, 80, 100, 120)):
            ka_setup([office, cafe])
            ka_run(60)
            n0 = len(sv_lines())
            for m in last:
                hb_fix(m, 0, speed=0.0)
            WLAN.scan_aps = []
            ka_run(3 * 60)
            res.append((not gl.log["still"]["active"], sv_lines()[n0:]))
    finally:
        WLAN.scan_aps, gl.PB_MODE = saved_aps, saved_mode
        gl.wifi.update(wlan=None, state="offline", nets=[])
    check("STAYWIFI: left near the anchor: the stay ends, no drift written",
          res[0][0] and res[0][1] == [], repr(res[0]))
    check("STAYWIFI: left out past STILL_NEAR: the way out is written",
          res[1][0] and len(res[1][1]) >= 3
          and sv_m(res[1][1][-1]) > 110, repr(res[1]))

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

    # GAPFAR: an away stay, no fix for 2 min (a metro, a garage), then
    # sitting still elsewhere: nothing recedes, so the stay never ended and
    # nothing was written there (no heartbeat either: last sample far out).
    # Own random source: later tests depend on rnd's sequence.
    grnd = random.Random(2)

    def g_drift(seconds, east=0.0, box=10.0, step=3.0):
        e = n = 0.0
        for _ in range(seconds):
            e = max(-box, min(box, e + grnd.uniform(-step, step)))
            n = max(-box, min(box, n + grnd.uniform(-step, step)))
            sv_fix(east + e, n, 1.0)

    sv_reset()
    g_drift(150)
    ok0 = sv_active()
    for _ in range(120):
        hb_fix(0, 0, nofix=True, speed=0.0)
    n0 = len(sv_lines())
    g_drift(10 * 60, east=3000.0, box=1.0, step=0.5)   # outdoors: still
    a = gl.log["still"]["anchor"]
    there = [l for l in sv_lines()[n0:] if sv_m(l) > 2900]
    check("GAPFAR: back after a gap 3 km away, sitting still: the stay ends",
          ok0 and (not sv_active() or gl._dist(a[0], a[1], 47.5, 19.0) > 2900)
          and len(there) >= 1, "active=%s there=%d" % (sv_active(), len(there)))
    check("GAPFAR: ... the new place starts from the first fix there",
          bool(there) and there[0] == sv_lines()[n0], repr(there[:1]))
    # 200 m out, sitting still (no clear jump): STILL_MIN_FIXES fixes there
    # over STILL_GAP_STAY end it, the way out from the first of them.
    # (STILL_EXIT_N samples out there did: a jump frozen for 3 s too.)
    def g_gap():
        sv_reset()
        g_drift(150)
        ok = sv_active()
        for _ in range(120):
            hb_fix(0, 0, nofix=True, speed=0.0)
        return ok, len(sv_lines())

    gap_stay = getattr(gl, "STILL_GAP_STAY", 60000)    # (getattr: the
    gap_jump = getattr(gl, "STILL_GAP_JUMP", 1000)     # old code runs too)
    ok0, n0 = g_gap()
    states = []
    first = None
    for k in range(gap_stay // 1000 + 5):
        sv_fix(200.0 + grnd.uniform(-1, 1), grnd.uniform(-1, 1), 0.0)
        first = first or gl._tc()
        states.append(sv_active())
    left = states.index(False) if False in states else None
    new = sv_lines()[n0:]
    check("GAPFAR: 200 m out after a gap, sitting still: leaves after "
          "STILL_GAP_STAY", ok0 and left is not None
          and gap_stay // 1000 <= left
          <= gap_stay // 1000 + 1, "left at %s" % left)
    check("GAPFAR: ... the way out starts at the first fix there",
          bool(new) and abs(sv_m(new[0]) - 200) < 3
          and new[0].endswith(first + "\n"), "%r %s" % (new[:1], first))
    # the false departures the review found: one 320 m outlier, a jump
    # frozen 200 m out for 3 s, a cold start walking in from 230 m
    ok0, n0 = g_gap()
    sv_fix(320.0, 0, 0.0)
    g_drift(30)
    out = [l for l in sv_lines()[n0:] if sv_m(l) > gl.STILL_EXIT]
    check("GAPFAR: one 320 m outlier after a gap keeps the stay, no spike",
          ok0 and sv_active() and not out, "active=%s out=%s" % (
              sv_active(), out))
    ok0, n0 = g_gap()
    for _ in range(3):
        sv_fix(200.0, 0, 0.0)
    g_drift(30)
    out = [l for l in sv_lines()[n0:] if sv_m(l) > gl.STILL_EXIT]
    check("GAPFAR: a jump frozen 200 m out for 3 s after a gap keeps the stay",
          ok0 and sv_active() and not out, "active=%s out=%s" % (
              sv_active(), out))
    ok0, n0 = g_gap()
    for m in range(230, 0, -15):                  # converging, 15 m a fix
        sv_fix(float(m), 0, 0.0)
    g_drift(30)
    out = [l for l in sv_lines()[n0:] if sv_m(l) > gl.STILL_EXIT]
    check("GAPFAR: a cold start walking in from 230 m keeps the stay",
          ok0 and sv_active() and not out, "active=%s out=%s" % (
              sv_active(), out))
    # ... and what still leaves after a gap: walking on away (a receding
    # run), indoor drift at a new place 400 m away (fixes coming closer
    # every other second are not counted, nor do they start over)
    ok0, n0 = g_gap()
    left = None
    for k in range(60):
        sv_fix(160.0 + k * 1.4, 0, 5.0)
        if left is None and not sv_active():
            left = k + 1
    check("GAPFAR: walking on away after a gap leaves (receding run)",
          ok0 and left is not None and left <= gl.STILL_EXIT_SLOW + 1,
          "left after %s" % left)
    ok0, n0 = g_gap()
    states = []
    for k in range(3 * 60):
        sv_fix(400.0 + grnd.uniform(-35, 35), grnd.uniform(-35, 35), 0.0)
        states.append(sv_active())
    left = states.index(False) if False in states else None
    check("GAPFAR: indoor drift 400 m away after a gap ends the stay",
          ok0 and left is not None and left <= 2 * gap_stay // 1000,
          "left at %s" % left)
    ok0, n0 = g_gap()
    sv_fix(gap_jump + 50.0, 0, 0.0)
    check("GAPFAR: one fix past STILL_GAP_JUMP after a gap ends it at once",
          ok0 and not sv_active())
    # a gap with the fix back at the stay keeps it
    sv_reset()
    g_drift(150)
    for _ in range(120):
        hb_fix(0, 0, nofix=True, speed=0.0)
    g_drift(60)
    check("GAPFAR: a gap with the fix back at the place keeps the stay",
          sv_active())

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

    # SEG2: two short losses while the lines are held (a walk) are two
    # breaks; the second one used to merge into the first, still unwritten
    sv_reset()
    x = 0.0
    for part in (30, 10, 10):
        if x:
            for _ in range(40):
                hb_fix(0, 0, nofix=True, speed=5.0)  # 40 s without a fix
                x += 1.4
        for _ in range(part):
            sv_fix(x, 0, 5.0)
            x += 1.4
    held = len(gl.log["batch"])
    gl.flush()
    with open(gl.log["file"]) as fh:
        body = [l for l in fh if not l.startswith("#")]
    marks = [i for i, l in enumerate(body) if l == "-\n"]
    check("SEG2: two short losses within held lines give two breaks",
          held >= 40 and marks == [30, 41], "held=%d marks=%s" % (held, marks))
    # SEGCAP: breaks queued while writes fail (or lines are held) with a
    # flapping fix: a cap of 16 dropped the oldest, 18 losses wrote 16
    # breaks. Breaks whose lines the MAX_BATCH cap dropped go before the
    # oldest line left, one for all, so the queue stays within the batch.
    def seg_run(losses):
        sv_reset()
        gl.start_file("20261029_060000")
        top = 0

        def rec(k):
            gl._record(("x", "47.500000,%.6f,,,20261029T%02d%02d%02dZ\n"
                        % (19.0 + k * 1e-4, 6 + k // 3600, k // 60 % 60,
                           k % 60)), (47.5, 19.0))
        rec(0)
        for k in range(1, losses + 1):
            gl._seg_break()
            rec(k)
            top = max(top, len(gl.log["seg_tc"]))
        gl.flush()
        with open(gl.log["file"]) as fh:
            body = [l for l in fh if not l.startswith("#")]
        return body, top

    body, top = seg_run(18)
    marks = [i for i, l in enumerate(body) if l == "-\n"]
    check("SEGCAP: 18 losses before a write give 18 breaks",
          len(marks) == 18 and all(body[m + 1] != "-\n" for m in marks)
          and marks[0] == 1, "marks=%s" % marks)
    body, top = seg_run(gl.MAX_BATCH + 60)
    marks = [i for i, l in enumerate(body) if l == "-\n"]
    check("SEGCAP: breaks of lines the batch cap dropped merge into one",
          len(body) - len(marks) == gl.MAX_BATCH
          and len(marks) == gl.MAX_BATCH - 1 and body[0] != "-\n"
          and top <= gl.MAX_BATCH + 1, "lines=%d marks=%d top=%d" % (
              len(body) - len(marks), len(marks), top))

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
    # (at a flush tick the new trip went on the moved lines without a break)
    first_new = gl.log["batch"][-1]
    gl.flush()
    with open(gl.log["file"]) as fh:
        body = [l for l in fh if not l.startswith("#")]
    check("FIXGAP: the new trip goes after a break, not on the moved lines",
          body.count("-\n") == 1 and body[-1] == first_new
          and body[-2] == "-\n", repr(body[-3:]))
    # FIXGAP: the gap is judged once. The fix back after NEW_FILE_GAP ran
    # _still_gap before the old track closed, and again from _sample with
    # last_at unchanged: the first verdict (from the open window, 95 m off)
    # held the previous window over to the new file, the second (from that
    # window, 185 m off) released it there. The old place's drift belongs
    # to the old track.
    sv_reset()
    gl.wifi.update(state="offline")
    crnd = random.Random(5)
    for i in range(30):
        sv_fix(-300 + i * 10.0, 0, 20.0)            # arriving
    for _ in range(92):                             # two windows here
        sv_fix(crnd.uniform(-5, 5), crnd.uniform(-5, 5), 1.0)
    w3 = gl.log["still"]["win_at"]
    for _ in range(30):                             # the third 90 m off
        sv_fix(90 + crnd.uniform(-5, 5), crnd.uniform(-5, 5), 1.0)
    trip = gl.log["file"]
    before = [l for l in gl.log["batch"] if sv_m(l) < 30]
    for _ in range(gl.NEW_FILE_GAP + 10):
        hb_fix(0, 0, nofix=True, speed=0.0)
    for _ in range(70):
        sv_fix(185 + crnd.uniform(-1, 1), 0, 1.0)
    gl.flush()
    with open(trip) as fh:
        trip_body = fh.read()
    later = [l for n in sorted(os.listdir(rv_dir))
             if n.endswith(".trk") and os.path.join(rv_dir, n) != trip
             for l in open(os.path.join(rv_dir, n)) if sv_m_ok(l)
             and sv_m(l) < 30]
    check("FIXGAP: the old place's drift before the gap stays in its track",
          w3 is None and not sv_active() and before
          and all(l in trip_body for l in before) and not later,
          "held=%d in trip=%d in later files=%d" % (
              len(before), sum(l in trip_body for l in before), len(later)))
    # a fix back elsewhere right after a flush: the drive held by its open
    # window kept the batch non-empty, so the new trip went into the old
    # file after a "-"; a loss over NEW_FILE_GAP is a new file
    sv_reset()
    gl.wifi.update(state="offline")
    for i in range(20):
        sv_fix(i * 6.0, 0, 22.0)
    trip = gl.log["file"]
    for _ in range(gl.NEW_FILE_GAP + 10):
        hb_fix(0, 0, nofix=True)
    gl.log["last_flush"] = _clock[0]                # a flush just happened
    sv_fix(500, 0, 22.0)
    trip_pts = []
    if os.path.exists(trip):
        with open(trip) as fh:
            trip_pts = [l for l in fh if l.count(",") == 4]
    check("FIXGAP: a loss over NEW_FILE_GAP starts a new file, flush or not",
          trip in gl.pending() and len(trip_pts) == 20
          and gl.log["file"] not in (None, trip) and not gl.log["new_seg"]
          and gl.log["prev"] is not None,
          "file=%s pts=%d new_seg=%s" % (gl.log["file"], len(trip_pts),
                                         gl.log["new_seg"]))
    # WFAIL: a write failure belongs to the batch and file it hit: a track
    # closed with nothing left to write (the batch dropped by a stay or a
    # full flash) or a new track starts clean, so a fix back after
    # NEW_FILE_GAP within FLUSH still closes the next one
    sv_reset()
    gl.start_file("20261018_090000")
    gl.log["wfail"] = True
    closed = gl._close_file()
    w_close = gl.log["wfail"]
    gl.log["wfail"] = True
    gl.start_file("20261018_090100")
    check("WFAIL: a closed or a new track clears the old write failure",
          closed and w_close is False and gl.log["wfail"] is False,
          "close=%s new=%s" % (w_close, gl.log["wfail"]))

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
        # WEB_AWAY=1: the page keeps the radio on on the other networks too
        gl.WEB_AWAY = True
        sv_reset()
        w, calls, out = rv_start_wifi(rv_env, [("phone", -40)], ["phone"])
        aw_conn = (gl.wifi["state"] == "connected"
                   and gl.wifi["ssid"] == "phone" and not gl.wifi["home"])
        for _ in range(5):
            _clock[0] += gl.WIFI_IDLE
            gl.wifi_tick()
        web_away_on = (gl.wifi["state"] if aw_conn else "not connected",
                       wl().active())
        gl.WEB, gl.WEB_AWAY = False, True
        sv_reset()
        w, calls, out = rv_start_wifi(rv_env, [("phone", -40)], ["phone"])
        _clock[0] += gl.WIFI_IDLE
        gl.wifi_tick()
        away_no_web = gl.wifi["state"]
    finally:
        gl.WEB = False
        gl.WEB_AWAY = False
    check("WEB: at home, everything uploaded, the radio stays connected",
          web_home, "state=%s" % gl.wifi["state"])
    check("WEB: on a non-home network it still sleeps once uploaded",
          web_away == "sleep", web_away)
    check("WEB: after WIFI_SLEEP a round runs even with nothing to upload",
          web_hourly)
    check("WEB_AWAY: on a non-home network the radio stays connected",
          web_away_on == ("connected", True), repr(web_away_on))
    check("WEB_AWAY: without WEB the radio still sleeps away",
          away_no_web == "sleep", away_no_web)

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
    saved_page = gl.WEB_PAGE
    gl.WEB_PAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "..", "web.html")
    with open(gl.WEB_PAGE, "rb") as f:
        page_file = f.read()
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
        reqs = [FakeConn(b"GET / HTTP/1.1\r\nHost: 192.168.1.60\r\n\r\n"),
                FakeConn(b"GET /status.json HTTP/1.1\r\n\r\n"),
                FakeConn(b"GET /nope HTTP/1.1\r\n\r\n"),
                FakeConn(b"", fail=True)]
        gl.WEB_PAGE = gl.WEB_PAGE + ".missing"
        nopage = FakeConn(b"GET / HTTP/1.1\r\n\r\n")
        lst.waiting.append(nopage)
        gl.web_tick()
        gl.WEB_PAGE = gl.WEB_PAGE[:-len(".missing")]
        for c in reqs:
            lst.waiting.append(c)
            gl.web_tick()
        gl.web_tick()                        # nobody waiting: no error
        # WEB-GZIP: web.html.gz (deploy.sh) to a client accepting gzip
        import gzip
        gz_td = tempfile.mkdtemp(prefix="webgz-")
        gz_file = gzip.compress(page_file)
        with open(os.path.join(gz_td, "web.html.gz"), "wb") as f:
            f.write(gz_file)
        saved_gz = getattr(gl, "WEB_PAGE_GZ", None)
        gl.WEB_PAGE_GZ = os.path.join(gz_td, "web.html.gz")
        gz_reqs = [
            FakeConn(b"GET / HTTP/1.1\r\nHost: 192.168.1.60\r\n"
                     b"Accept-Encoding: gzip, deflate, br\r\n\r\n"),
            FakeConn(b"GET /?x HTTP/1.1\r\naccept-encoding: GZIP\r\n\r\n"),
            FakeConn(b"GET / HTTP/1.1\r\nAccept-Encoding: identity\r\n"
                     b"X-Note: gzip\r\n\r\n"),
            FakeConn(b"GET / HTTP/1.1\r\n\r\n")]
        for c in gz_reqs:
            lst.waiting.append(c)
            gl.web_tick()
        gl.WEB_PAGE_GZ = os.path.join(gz_td, "missing.gz")
        gz_none = FakeConn(b"GET / HTTP/1.1\r\nAccept-Encoding: gzip\r\n\r\n")
        lst.waiting.append(gz_none)
        gl.web_tick()
        if saved_gz is None:
            del gl.WEB_PAGE_GZ
        else:
            gl.WEB_PAGE_GZ = saved_gz
        shutil.rmtree(gz_td, ignore_errors=True)
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
        aways = []
        for start, v in ((False, "1"), (True, "0"), (True, "maybe"),
                         (False, None)):
            gl.WEB_AWAY = start
            gl._web_env({"WEB_AWAY": v} if v is not None else {})
            aways.append(gl.WEB_AWAY)
    finally:
        gl._web_close()
        gl.socket = real_socket
        gl.WEB_PAGE = saved_page
        gl.WEB = False
        gl.WEB_AWAY = False
        gl.wifi["state"] = "sleep"
    check("WEB: listens on port 80, non-blocking, while connected", opened)
    check("WEB: GET / sends web.html from flash with its Content-Length",
          head.startswith(b"HTTP/1.0 200 OK") and b"text/html" in head
          and (b"Content-Length: %d" % len(page_file)) in head
          and html == page_file and reqs[0].closed
          and reqs[0].timeout == gl.WEB_TIMEOUT, repr(reqs[0].sent[:300]))
    gz_got = [c.sent.partition(b"\r\n\r\n") for c in gz_reqs + [gz_none]]
    check("WEB-GZIP: Accept-Encoding gzip (any case) gets web.html.gz, "
          "Content-Encoding and its length",
          all(h.startswith(b"HTTP/1.0 200 OK") and b"text/html" in h
              and b"\r\nContent-Encoding: gzip\r\n" in h
              and (b"Content-Length: %d\r\n" % len(gz_file)) in h
              and b == gz_file for h, _, b in gz_got[:2]),
          repr([g[0][:200] for g in gz_got[:2]]))
    check("WEB-GZIP: no gzip in Accept-Encoding, or no .gz on the board -> "
          "the plain page",
          all(b"Content-Encoding" not in h and b == page_file
              and (b"Content-Length: %d\r\n" % len(page_file)) in h
              for h, _, b in gz_got[2:]),
          repr([g[0][:200] for g in gz_got[2:]]))
    check("WEB: no web.html on the board -> 404 naming deploy.sh and the JSON",
          nopage.sent.startswith(b"HTTP/1.0 404") and b"deploy.sh" in
          nopage.sent and b"/status.json" in nopage.sent,
          repr(nopage.sent[:200]))
    check("WEB: /status.json has the same values and the diag tail",
          jhead.startswith(b"HTTP/1.0 200 OK") and jd.get("points") == 42
          and jd.get("left") == 7 and jd.get("sent") == 35
          and "fix" in jd and "sats" in jd and jd.get("refresh_s") == 15
          and jd.get("wifi") == "connected <b>home"
          and jd.get("diag", [""])[-1].startswith("up 60"), repr(jd)[:300])
    check("WEB: unknown path 404, a failing client is closed, no raise",
          reqs[2].sent.startswith(b"HTTP/1.0 404") and reqs[3].closed
          and reqs[3].sent == b"")
    check("WEB: the server closes when the radio sleeps; WEB=0 opens none",
          closed_on_sleep and off_no_socket)
    check("WEB: .env WEB 0/1, anything else keeps the setting",
          envs == [False, True, True], repr(envs))
    check("WEB_AWAY: .env 1/0, invalid or missing keeps the setting",
          aways == [True, False, True, False], repr(aways))
    import inspect
    check("WEB: _loop serves the page; wifi_stop closes it",
          "web_tick()" in inspect.getsource(gl._loop)
          and "_web_close()" in inspect.getsource(gl.wifi_stop))

    # WEB-DEADLINE: one wall-clock limit per request (a client that sends
    # or reads a byte at a time held the loop ~50 s: 2 s per recv/send);
    # WEB-GET: only a GET is answered (an empty request got the 21 KB page);
    # WEB-HOST: DNS rebinding guard on the Host header
    class SlowConn(FakeConn):
        def __init__(self, req, slow_recv=False, slow_send=False):
            FakeConn.__init__(self, req)
            self.slow_recv, self.slow_send, self.timeouts = (
                slow_recv, slow_send, [])

        def settimeout(self, t):
            self.timeouts.append(t)

        def recv(self, n):
            if self.slow_recv:           # one byte a second
                _clock[0] += 1000
                n = 1
            b, self.req = self.req[:n], self.req[n:]
            return b

        def sendall(self, b):
            if self.slow_send:           # every send waits its whole timeout
                _clock[0] += int(self.timeouts[-1] * 1000)
            self.sent += bytes(b)

    gl.socket = FakeSocketMod
    gl.WEB = True
    gl.WEB_PAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "..", "web.html")
    gl.wifi.update(state="connected", ip="192.168.1.60")
    good = b"GET / HTTP/1.1\r\nHost: 192.168.1.60\r\n\r\n"

    def serve(c):
        lst = gl._web_sock or (gl.web_tick() or gl._web_sock)
        lst.waiting.append(c)
        t0 = _clock[0]
        gl.web_tick()
        return _clock[0] - t0

    try:
        gl._web_close()
        slow_in = SlowConn(good, slow_recv=True)
        t_in = serve(slow_in)
        slow_out = SlowConn(good, slow_send=True)
        t_out = serve(slow_out)
        empty, post = FakeConn(b""), FakeConn(b"POST / HTTP/1.1\r\n\r\n")
        serve(empty)
        serve(post)
        hosts = {}
        for h in (b"evil.example", b"192.168.1.60:80", b"192.168.1.60",
                  b"dawarich-gps-tracker.local", b"Dawarich-GPS-tracker",
                  b"DAWARICH-GPS-TRACKER.LOCAL.", b"192.168.1.60.evil.example",
                  b"evil.example:80", None):
            c = FakeConn(b"GET /status.json HTTP/1.1\r\n"
                         + (b"Host: " + h + b"\r\n" if h else b"")
                         + b"Accept: */*\r\n\r\n")
            serve(c)
            hosts[h] = c.sent.split(b"\r\n", 1)[0]
        low = FakeConn(b"GET / HTTP/1.1\r\nhost: evil.example\r\n\r\n")
        serve(low)
    finally:
        gl._web_close()
        gl.socket = real_socket
        gl.WEB_PAGE = saved_page
        gl.WEB = False
        gl.wifi["state"] = "sleep"
    check("WEB-DEADLINE: a client sending a byte a second is dropped at "
          "WEB_DEADLINE, unanswered",
          t_in <= gl.WEB_DEADLINE * 1000 and slow_in.sent == b""
          and slow_in.closed, "%d ms %r" % (t_in, slow_in.sent[:60]))
    check("WEB-DEADLINE: a slow reader gets at most WEB_DEADLINE s of sends, "
          "timeouts cut to what is left",
          t_out <= gl.WEB_DEADLINE * 1000
          and 0 < len(slow_out.sent) < len(page_file)
          and max(slow_out.timeouts) <= gl.WEB_TIMEOUT
          and min(slow_out.timeouts) < gl.WEB_TIMEOUT and slow_out.closed,
          "%d ms %d B %r" % (t_out, len(slow_out.sent), slow_out.timeouts))
    check("WEB-GET: an empty or non-GET request is closed without a reply",
          empty.sent == b"" and empty.closed and post.sent == b""
          and post.closed, repr((empty.sent[:60], post.sent[:60])))
    ok200, no = b"HTTP/1.0 200 OK", b"HTTP/1.0 403 Forbidden"
    check("WEB-HOST: Host must be the board's IP, HOSTNAME[.local] or none",
          hosts[b"evil.example"] == no and hosts[b"evil.example:80"] == no
          and hosts[b"192.168.1.60.evil.example"] == no
          and low.sent.startswith(no) and b"192.168.1.60" in low.sent
          and all(hosts[h] == ok200 for h in (
              b"192.168.1.60:80", b"192.168.1.60", None,
              b"dawarich-gps-tracker.local", b"Dawarich-GPS-tracker",
              b"DAWARICH-GPS-TRACKER.LOCAL.")), repr(hosts))

    # SATS: the satellite list from UBX-NAV-SVINFO, read out of the stream
    import struct

    def svinfo(chans):
        pl = struct.pack("<IBBH", 1000, len(chans), 0, 0)
        for k, (svid, flags, cno, el, az) in enumerate(chans):
            pl += struct.pack("<BBBBBbhi", k, svid, flags, 4, cno, el, az, 0)
        return gl._ubx(0x01, 0x30, pl)

    chans = [(12, 0x01, 38, 54, 10),         # used; azimuth 10 = a 0x0A byte
             (5, 0x01, 44, 71, 200),
             (25, 0x00, 0, 8, 300),          # no signal
             (123, 0x10, 33, 30, 160)]       # SBAS, unhealthy
    frame = svinfo(chans)
    gl.reset()
    _uart_rx[:] = []
    rmc = nmea("GPRMC,120500.00,V,,,,,,,171026,,,N") + b"\r\n"
    data = rmc + frame + rmc
    half = len(rmc) + len(frame) // 2
    _uart_rx.extend(data[:half])
    gl.read_uart()
    _uart_rx.extend(data[half:])
    gl.read_uart()
    check("SATS: NAV-SVINFO is read out of the NMEA stream, 0x0A and all",
          b"\n" in frame[6:-2] and gl.st["svs"] is not None
          and [v[0] for v in gl.st["svs"]] == [12, 5, 25, 123]
          and gl.st["svs"][0] == (12, 38, 54, 10, True, False, 0)
          and gl.st["svs"][3][5] is True
          and gl.st["ok"] == 2 and gl.st["err"] == 0,
          "svs=%r ok=%d err=%d" % (gl.st["svs"], gl.st["ok"], gl.st["err"]))
    check("SATS: seen counts the channels with a signal",
          gl.sv_seen() == 3, gl.sv_seen())
    bad = bytearray(frame)
    bad[-1] ^= 0xFF
    old_svs = gl.st["svs"]
    ack = b"\xb5\x62\x05\x01\x02\x00\x06\x01\x0f\x38"
    _uart_rx.extend(bytes(bad) + ack + rmc)
    gl.read_uart()
    check("SATS: a broken frame is an error, a late ACK is just dropped",
          gl.st["svs"] is old_svs and gl.st["err"] == 1 and gl.st["ok"] == 3,
          "err=%d ok=%d" % (gl.st["err"], gl.st["ok"]))

    gl.reset()
    gl.handle(gsv_line)
    seen_gsv = gl.sv_seen()
    _uart_tx[:] = []
    gl.gps_setup()
    check("SATS: before the setup GSV counts; a good setup drops that count",
          seen_gsv == 12 and gl.st["view"] == {} and gl.sv_seen() == 0,
          "%d %r" % (seen_gsv, gl.st["view"]))

    def polls():
        return len([f for f in _uart_tx if f[:4] == b"\xb5\x62\x01\x30"])

    _uart_tx[:] = []
    for _ in range(3):
        gl.gps_check(_clock[0])
        _clock[0] += gl.SV_POLL // 2
    p_ok = polls()
    gl.st["setup_ok"] = False
    _uart_tx[:] = []
    _clock[0] += gl.SV_POLL
    gl.gps_check(_clock[0])
    p_nosetup = polls()
    gl.st["setup_ok"] = True
    check("SATS: polled every SV_POLL, also while the setup is not acked",
          p_ok == 2 and p_nosetup == 1, "%d %d" % (p_ok, p_nosetup))
    check("SATS: the poll frame has a valid checksum",
          gl._ubx(0x01, 0x30, b"") == b"\xb5\x62\x01\x30\x00\x00\x31\x94")

    gl.reset()
    gl.st["setup_ok"] = True
    _uart_rx[:] = []
    _uart_tx[:] = []
    _uart_sv[0] = frame
    try:
        c = FakeConn(b"GET /status.json HTTP/1.1\r\n\r\n")
        gl._web_reply(c, c.req)
        jd0 = json.loads(c.sent.partition(b"\r\n\r\n")[2])
        p_page = polls()
        c2 = FakeConn(b"GET /status.json HTTP/1.1\r\n\r\n")
        gl._web_reply(c2, c2.req)
        p_again = polls()
        jd = json.loads(c2.sent.partition(b"\r\n\r\n")[2])
        _uart_sv[0] = None
        gl.reset()
        t0 = _clock[0]
        c3 = FakeConn(b"GET /status.json HTTP/1.1\r\n\r\n")
        gl._web_reply(c3, c3.req)
        waited = _clock[0] - t0
        jd3 = json.loads(c3.sent.partition(b"\r\n\r\n")[2])
    finally:
        _uart_sv[0] = None
    names = [v["name"] for v in jd0["satellites"]]
    check("SATS: a page poll asks the receiver and lists the satellites",
          p_page == 1 and jd0["satellites_age_s"] == 0
          and names == ["GPS 5", "GPS 12", "SBAS 123", "GPS 25"]
          and jd0["satellites"][2]["unhealthy"] is True, repr(names))
    check("SATS: a list younger than SV_FRESH is not asked for again; JSON",
          p_again == 1 and jd["satellites"][0]["name"] == "GPS 5"
          and jd["satellites"][1] == {"name": "GPS 12", "system": "GPS",
                                      "svid": 12, "cno": 38, "elev": 54,
                                      "azim": 10, "used": True,
                                      "unhealthy": False}
          and len(jd["satellites"]) == 4, repr(jd.get("satellites"))[:300])
    check("SATS: a silent receiver costs at most SV_WAIT, the list is empty",
          waited <= gl.SV_WAIT + 100 and jd3["satellites"] == []
          and jd3["satellites_age_s"] is None, "waited=%d" % waited)

    # M10: no NAV-SVINFO on a u-blox M10 (seen 2026-10-29: SPG 5.10 does
    # not answer it, the page kept the old receiver's list); NAV-SAT instead,
    # names per system, the receiver model from MON-VER
    def navsat(svs):
        pl = struct.pack("<IBBH", 2000, 1, len(svs), 0)
        for gnss, svid, cno, el, az, flags in svs:
            pl += struct.pack("<BBBbhhI", gnss, svid, cno, el, az, 0, flags)
        return gl._ubx(0x01, 0x35, pl)

    def monver(sw, hw, ext):
        pl = sw.ljust(30, b"\0") + hw.ljust(10, b"\0")
        for e in ext:
            pl += e.ljust(30, b"\0")
        return gl._ubx(0x0A, 0x04, pl)

    m10 = [(0, 7, 41, 60, 270, 0x1F),       # GPS, used (bit 3), healthy
           (2, 11, 37, 45, 120, 0x1F),      # Galileo, used
           (3, 22, 30, 20, 10, 0x2C),       # BeiDou, used, unhealthy
           (6, 3, 0, 5, 300, 0x10),         # GLONASS, no signal
           (3, 33, 0, -91, 0, 0x10)]        # BeiDou, position not known
    sat_frame = navsat(m10)
    ver_frame = monver(b"ROM SPG 5.10 (7b202e)", b"000A0000",
                       [b"FWVER=SPG 5.10", b"PROTVER=34.10",
                        b"GPS;GLO;GAL;BDS", b"SBAS;QZSS"])

    def polls_of(mid):
        return len([f for f in _uart_tx if f[:4] == b"\xb5\x62" + mid])

    gl.reset()
    gl.st["setup_ok"] = True
    _uart_rx[:] = []
    _uart_tx[:] = []
    _uart_sat[0], _uart_ver[0] = sat_frame, ver_frame
    try:
        c = FakeConn(b"GET /status.json HTTP/1.1\r\n\r\n")
        gl._web_reply(c, c.req)
        for _ in range(3):                  # the answers after SV_WAIT too
            gl.read_uart()
        first = (polls_of(b"\x01\x30"), polls_of(b"\x01\x35"),
                 polls_of(b"\x0a\x04"))
        _uart_tx[:] = []
        _clock[0] += gl.SV_POLL
        gl.gps_check(_clock[0])
        later = (polls_of(b"\x01\x30"), polls_of(b"\x01\x35"),
                 polls_of(b"\x0a\x04"))
        gl.read_uart()
        c2 = FakeConn(b"GET /status.json HTTP/1.1\r\n\r\n")
        _clock[0] += gl.SV_FRESH
        gl._web_reply(c2, c2.req)
        jd = json.loads(c2.sent.partition(b"\r\n\r\n")[2])
        m10_svs = gl.st["svs"]
        seen_m10 = gl.sv_seen()
        try:
            with open(gl.DIAG_FILE) as f:
                diag_m10 = f.read()
        except OSError:
            diag_m10 = ""
        # a receiver restart (or swap) forgets the list and the model
        gl.handle(nmea("GPTXT,01,01,02,u-blox ag - www.u-blox.com"))
        after_boot = (gl.st["svs"], gl.st.get("sv_msg", "-"),
                      gl.st.get("gps_ver", "-"))
    finally:
        _uart_sat[0] = _uart_ver[0] = None
    check("M10: no SVINFO answer -> NAV-SAT is read; per-system satellites",
          m10_svs is not None and [(v[6], v[0]) for v in m10_svs]
          == [(0, 7), (2, 11), (3, 22), (6, 3), (3, 33)]
          and m10_svs[4][2] is None
          and m10_svs[2][4] is True and m10_svs[2][5] is True
          and m10_svs[3][4] is False and m10_svs[0][5] is False
          and seen_m10 == 3, "%r seen=%r" % (m10_svs, seen_m10))
    m10_names = [(v["name"], v["unhealthy"], v["elev"])
                 for v in jd["satellites"]]
    check("M10: JSON lists GPS/Galileo/BeiDou/GLONASS names",
          m10_names == [("GPS 7", False, 60), ("Galileo 11", False, 45),
                        ("BeiDou 22", True, 20), ("BeiDou 33", False, None),
                        ("GLONASS 3", False, 5)], repr(m10_names))
    check("M10: the receiver model from MON-VER is in the JSON",
          jd.get("receiver") == "u-blox M10, ROM SPG 5.10 (7b202e), "
                                    "PROTVER=34.10"
          and jd["satellites"][1]["system"] == "Galileo"
          and "gps u-blox M10, ROM SPG 5.10" in diag_m10,
          repr(jd.get("receiver")))
    check("M10: first both list polls and MON-VER, then NAV-SAT only",
          first == (1, 1, 1) and later == (0, 1, 0),
          "first=%r later=%r" % (first, later))
    check("M10: a receiver restart forgets the list and the model",
          after_boot == (None, None, None), repr(after_boot))

    # M10-VER: MON-VER is asked for again after every receiver restart, and
    # each answer wrote a diag line (a reset storm: one per restart); now
    # one only when the model changed
    def ver_diag():
        try:
            with open(gl.DIAG_FILE) as f:
                return [ln for ln in f.read().splitlines()
                        if ln.startswith("gps u-blox") or ln.startswith("gps hw")]
        except OSError:
            return []

    saved_d = gl.DIAG
    gl.DIAG = 60000
    gl.reset()
    v0 = len(ver_diag())
    other = monver(b"ROM SPG 5.10 (7b202e)", b"00190000", [b"PROTVER=18.00"])
    for fr in (ver_frame, ver_frame, ver_frame, other, ver_frame):
        gl._ubx_frame(fr)
        gl.handle(nmea("GPTXT,01,01,02,u-blox ag - www.u-blox.com"))
    ver_lines = ver_diag()[v0:]
    gl.DIAG = saved_d
    check("M10-VER: a diag line for MON-VER only when the model changed",
          len(ver_lines) == 3 and ver_lines[0] == ver_lines[2]
          and ver_lines[0].startswith("gps u-blox M10, ROM SPG 5.10")
          and ver_lines[1] != ver_lines[0], repr(ver_lines))

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
    # WR-d: one failed clock change (the drop-back after a POST) is tried
    # again, not blacklisted for good (the board stayed at 160 MHz)
    mach = sys.modules["machine"]
    real_freq = mach.freq
    fails = [1]

    def flaky_freq(v=None):
        if v is not None and fails[0]:
            fails[0] -= 1
            raise OSError("busy")
        return real_freq(v)
    mach.freq = flaky_freq
    try:
        gl._freq_bad[:] = []
        real_freq(160000000)
        gl._set_freq(80)
        dropped = _freqs[-1] == 80000000 and 80 not in gl._freq_bad
        fails[0] = 99
        gl._set_freq(160)
        capped = 160 in gl._freq_bad and fails[0] >= 99 - 5
    finally:
        mach.freq = real_freq
        gl._freq_bad[:] = []
    check("WR-d: a failed clock change is retried, refused ones capped",
          dropped and capped, "%s %s %r" % (dropped, capped, _freqs[-3:]))
    # R2-d: a clock refused FREQ_TRIES times (a passing failure of the
    # drop-back after a POST) is tried again after FREQ_RETRY: it stayed
    # blacklisted until reboot, the board pinned at the upload clock
    fails = [0]
    mach.freq = flaky_freq
    try:
        gl._freq_bad[:] = []
        real_freq(160000000)
        fails[0] = gl.FREQ_TRIES
        gl._set_freq(80)
        stuck = _freqs[-1] == 160000000 and 80 in gl._freq_bad
        _clock[0] += 60000
        gl._set_freq(80)                 # within the hold: left alone
        held = _freqs[-1] == 160000000
        _clock[0] += getattr(gl, "FREQ_RETRY", 1800000)
        gl._set_freq(80)
        back = _freqs[-1] == 80000000 and 80 not in gl._freq_bad
    finally:
        mach.freq = real_freq
        gl._freq_bad[:] = []
    check("R2-d: a refused clock is tried again after FREQ_RETRY",
          stuck and held and back, "%s %s %s %r" % (stuck, held, back,
                                                     _freqs[-3:]))

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
        before = gl.wifi.get("rounds")   # (a good scan listing none of
        gl.wifi_tick()                   # them tries none: no connect)
        drain_wifi()
        if gl.wifi.get("rounds") != before:
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
        # WR-b: a pin the board already uses (GPS, I2C, boot-bypass, the
        # C6's USB) is refused: pulsing it broke the GPS or the console
        saved_c6 = gl.IS_C6
        kept = []
        try:
            for v in (gl.GPS_TX, gl.GPS_RX, gl.I2C_SCL, gl.I2C_SDA, 25):
                gl.PB_PIN = 26
                gl._keepalive_env({"KEEPALIVE_PIN": str(v)})
                kept.append(gl.PB_PIN)
            gl.IS_C6 = True
            for v in (10, 12, 13):
                gl.PB_PIN = 2
                gl._keepalive_env({"KEEPALIVE_PIN": str(v)})
                kept.append(gl.PB_PIN)
            gl._keepalive_env({"KEEPALIVE_PIN": "3"})
            free = gl.PB_PIN
        finally:
            gl.IS_C6 = saved_c6
        check("WR-b: KEEPALIVE_PIN on a GPS/I2C/boot/USB pin keeps the default",
              kept == [26] * 5 + [2] * 3 and free == 3, "%r %r" % (kept, free))
        # R2-12: pins the chip itself needs: ESP32 strapping GPIO0 (download
        # mode), UART0 1/3, SPI flash 6-11 (driving one crashes the board);
        # C6 strapping 8/9, UART0 16/17, SPI flash 24-30
        saved_c6, saved_pin = gl.IS_C6, gl.PB_PIN
        bad_ok, good = [], []
        try:
            gl.IS_C6 = False
            for v in (0, 1, 3, 6, 7, 8, 9, 10, 11):
                gl.PB_PIN = 26
                gl._keepalive_env({"KEEPALIVE_PIN": str(v)})
                if gl.PB_PIN != 26:
                    bad_ok.append(("esp32", v))
            for v in (2, 4, 27, 32):
                gl.PB_PIN = 26
                gl._keepalive_env({"KEEPALIVE_PIN": str(v)})
                good.append(gl.PB_PIN)
            gl.IS_C6 = True
            for v in (8, 9, 16, 17, 24, 27, 30):
                gl.PB_PIN = 2
                gl._keepalive_env({"KEEPALIVE_PIN": str(v)})
                if gl.PB_PIN != 2:
                    bad_ok.append(("c6", v))
            for v in (3, 11, 18, 23):
                gl.PB_PIN = 2
                gl._keepalive_env({"KEEPALIVE_PIN": str(v)})
                good.append(gl.PB_PIN)
        finally:
            gl.IS_C6, gl.PB_PIN = saved_c6, saved_pin
        check("R2-12: KEEPALIVE_PIN on a strapping/flash/UART0 pin is refused "
              "per chip", bad_ok == []
              and good == [2, 4, 27, 32, 3, 11, 18, 23],
              "%r %r" % (bad_ok, good))
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

    # WR-11: a .env saved as ANSI (Notepad): UnicodeError at every boot ->
    # boot-fail -> REPL. Now read as bytes: a non-UTF-8 SSID/WPA2 stays
    # bytes (connect() takes them), any other such value is skipped
    ansi = os.path.join(rv_dir, "env_ansi")
    with open(ansi, "wb") as fh:
        fh.write(b"SSID=K\xe1v\xe9z\xf3\nWPA2=jelsz\xf3\nGEO_URL=https://d\xe9\n"
                 b"API_KEY=k\nDEVICE_ID=bike\n")
    crash, env = None, {}
    try:
        env = gl.load_env(ansi)
        nets = gl._load_nets(env)
    except Exception as e:
        crash, nets = e, []
    raw = b"K\xe1v\xe9z\xf3"
    ok_env = (crash is None and env.get("SSID") == raw
              and env.get("WPA2") == b"jelsz\xf3" and "GEO_URL" not in env
              and env.get("API_KEY") == "k" and env.get("DEVICE_ID") == "bike")
    got = None
    if nets:
        real_scan = WLAN.scan
        WLAN.scan = lambda self: [(raw, b"\x07" * 6, 1, -50, 4, False)]
        try:
            reset_wlan()
            WLAN.ok = {raw}
            gl.wifi_reset()
            gl.wifi["nets"] = nets
            gl.wifi["wlan"] = wa = WLAN(0)
            wa.active(True)
            gl._wifi_begin_round(wa)
            for _ in range(10):
                _clock[0] += 500
                gl.wifi_tick()
            got = (gl.wifi["state"], [c[:2] for c in WLAN.connects[:1]],
                   gl._wifi_line())
        except Exception as e:
            crash = e
        finally:
            WLAN.scan = real_scan
            gl.wifi_reset()
    check("WR-11: a non-UTF-8 .env neither raises nor loses the network",
          ok_env and got is not None and got[0] == "connected"
          and got[1] == [(raw, b"jelsz\xf3")]
          and got[2].startswith("WiFi K"), "%r %r %r" % (crash, env, got))

    # WR-c: quotes and a comment on a comment-safe key: comment off first,
    # then the quotes (IP="192.168.1.60" # static kept the quotes: DHCP)
    qenv = os.path.join(rv_dir, "env_quote")
    with open(qenv, "wb") as fh:
        fh.write(b'SSID=home\nIP="192.168.1.60"  # static\n'
                 b"GW='192.168.1.1' # router\nDEVICE_ID=\"bike\" # mine\n")
    qn = gl._load_nets(gl.load_env(qenv))
    check("WR-c: a quoted value with a comment loses both (static IP kept)",
          qn and qn[0]["ip"] == ("192.168.1.60", "255.255.255.0",
                                 "192.168.1.1", "192.168.1.1")
          and gl._plain('"bike" # mine') == "bike",
          repr(qn and qn[0]["ip"]))

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
    try:
        out = subprocess.check_output(
            [mpy_bin, os.path.join(ROOT, "tests", "mpy_unix_logger.py")],
            stderr=subprocess.STDOUT, timeout=60)
        text = out.decode() if isinstance(out, bytes) else out
        print(text.rstrip())
        check("mpy-unix: gps_logger imports and runs (smoke)",
              "mpy logger smoke passed" in text, text[-300:])
    except Exception as e:
        out = getattr(e, "output", b"") or b""
        check("mpy-unix: gps_logger smoke", False, repr(e) + " "
              + out.decode(errors="replace")[-400:])

else:
    print("SKIP  mpy-unix (no micropython binary)")

# Optional: a single-precision unix build, floats as on the board (ESP32:
# MICROPY_FLOAT_IMPL_FLOAT). The smoke's departure ring checks then see what
# the board writes, which a double build cannot show. Build it with
#   make -C ports/unix BUILD=build-f32 PROG=micropython-f32 \
#     CFLAGS_EXTRA=-DMICROPY_FLOAT_IMPL=MICROPY_FLOAT_IMPL_FLOAT
mpy_f32 = os.environ.get("MICROPYTHON_F32")
if mpy_f32:
    import subprocess
    try:
        out = subprocess.check_output(
            [mpy_f32, "-c", "print(1.0 + 1e-12 == 1.0)"],
            stderr=subprocess.STDOUT, timeout=20)
        single = out.decode().strip() == "True"
        out = subprocess.check_output(
            [mpy_f32, os.path.join(ROOT, "tests", "mpy_unix_logger.py")],
            stderr=subprocess.STDOUT, timeout=60)
        text = out.decode()
        print(text.rstrip())
        check("mpy-unix-f32: gps_logger smoke on single-precision floats",
              single and "mpy logger smoke passed" in text,
              "single=%s %s" % (single, text[-300:]))
    except Exception as e:
        out = getattr(e, "output", b"") or b""
        check("mpy-unix-f32: gps_logger smoke", False, repr(e) + " "
              + out.decode(errors="replace")[-400:])
else:
    print("SKIP  mpy-unix-f32 (MICROPYTHON_F32 not set)")

# The same smoke under CPython (keeps the script itself working)
import subprocess
try:
    out = subprocess.check_output(
        [sys.executable, os.path.join(ROOT, "tests", "mpy_unix_logger.py")],
        stderr=subprocess.STDOUT, timeout=60)
    smoke = out.decode()
except subprocess.CalledProcessError as e:
    smoke = e.output.decode(errors="replace")
check("SMOKE: tests/mpy_unix_logger.py passes under CPython",
      "mpy logger smoke passed" in smoke, smoke[-300:])

# WEB-LOOPBACK: the status page over real loopback sockets (slow, stalled,
# empty, oversized, reset and deadline clients, gzip) on the unix port and
# CPython
import gzip
lb_td = tempfile.mkdtemp(prefix="loopgz-")
lb_gz = os.path.join(lb_td, "web.html.gz")
with open(os.path.join(ROOT, "web.html"), "rb") as f:
    lb_page = f.read()
with open(lb_gz, "wb") as f:
    f.write(gzip.compress(lb_page, 9))
for interp, tag in ((mpy_bin, "MicroPython unix"), (sys.executable, "CPython")):
    if not interp:
        print("SKIP  WEB-LOOPBACK on %s (no binary)" % tag)
        continue
    try:
        out = subprocess.check_output(
            [interp, os.path.join(ROOT, "tests", "web_loopback.py"), lb_gz],
            stderr=subprocess.STDOUT, timeout=60)
        loop_out = out.decode()
    except subprocess.CalledProcessError as e:
        loop_out = e.output.decode(errors="replace")
    except Exception as e:
        loop_out = repr(e)
    check("WEB-LOOPBACK: tests/web_loopback.py passes on " + tag,
          "web loopback passed" in loop_out, loop_out[-600:])
    check("WEB-LOOPBACK-GZIP: a client accepting gzip gets web.html.gz on "
          + tag, "ok   GET / with Accept-Encoding gzip" in loop_out,
          loop_out[-600:])
shutil.rmtree(lb_td, ignore_errors=True)

# DEPLOY: deploy.sh against a fake board (mpremote runs the board-side code
# under CPython in a directory): files go to <name>.new, are checked by size
# and SHA-256, then renamed over the old ones; low flash or a bad copy stops
# it with the old files untouched (a cut copy left an unimportable module)
FAKE_PY = r"""#!/usr/bin/env python3
import sys
a = sys.argv[1:]
if a[:1] == ["-c"]:
    sys.exit(0)
out = a[a.index("-o") + 1]
with open(a[-1], "rb") as f, open(out, "wb") as g:
    g.write(b"MPY" + f.read())
"""
FAKE_SHIM = r"""
import sys, os, types
sys.implementation._machine = "Generic ESP32C6 module with ESP32C6"
m = types.ModuleType("machine")
class RTC:
    def memory(self, b=None):
        open(".rtc", "wb").write(b)
m.RTC = RTC
sys.modules["machine"] = m
if os.environ.get("FAKE_FREE_BLOCKS"):
    _blocks = int(os.environ["FAKE_FREE_BLOCKS"])
    os.statvfs = lambda p: (4096, 4096, 512, _blocks, _blocks, 0, 0, 0, 0,
                            255)
"""
FAKE_MPREMOTE = r"""#!/usr/bin/env python3
import os, sys, json, subprocess
dev, log = os.environ["FAKE_DEV"], os.environ["FAKE_LOG"]
a = sys.argv[1:]
assert a[0] == "connect", a
cmds = [[]]
for x in a[2:]:
    if x == "+":
        cmds.append([])
    else:
        cmds[-1].append(x)
with open(log, "a") as f:
    f.write(json.dumps(cmds) + "\n")
shim = open(os.environ["FAKE_SHIM"]).read()
for c in cmds:
    if c[0] == "exec":
        if os.environ.get("FAKE_GARBLE") and "gps_frozen" in c[1]:
            print("could not enter raw repl")    # no frozen/plain answer
            sys.exit(1)
        r = subprocess.run([sys.executable, "-c", shim + c[1]], cwd=dev)
        if r.returncode:
            sys.exit(1)
    elif c[0] == "cp":
        with open(c[1], "rb") as f:
            data = f.read()
        if os.environ.get("FAKE_TRUNC") == c[1]:
            data = data[:len(data) // 2]
        with open(os.path.join(dev, c[2].lstrip(":")), "wb") as f:
            f.write(data)
    elif c[0] == "ls":
        print(sorted(os.listdir(dev)))
"""
dp_td = tempfile.mkdtemp(prefix="deploy-")


def deploy_run(env_extra):
    """deploy.sh in a fresh copy of the repo files against a fake board
    that holds an older install: (rc, stdout, stderr, board dir, calls)."""
    work = tempfile.mkdtemp(dir=dp_td)
    for n in ("deploy.sh", "gps_logger.py", "main.py", "web.html"):
        shutil.copy(os.path.join(ROOT, n), work)
    os.makedirs(os.path.join(work, ".venv", "bin"))
    for n, src in (("python", FAKE_PY), ("mpremote", FAKE_MPREMOTE),
                   ("shim.py", FAKE_SHIM)):
        path = os.path.join(work, ".venv", "bin", n)
        with open(path, "w") as f:
            f.write(src)
        os.chmod(path, 0o755)
    dev = os.path.join(work, "dev")
    os.makedirs(dev)
    for n, b in (("gps_logger.mpy", b"OLD MODULE"), ("main.py", b"old main"),
                 ("web.html", b"old page"), ("gps_logger.py", b"stray"),
                 ("web.html.new", b"left from a failed deploy")):
        with open(os.path.join(dev, n), "wb") as f:
            f.write(b)
    fz = env_extra.get("FAKE_FROZEN")           # firmware with the logger in:
    if fz:                                      # built from this gps_logger.py
        with open(os.path.join(work, "gps_logger.py"), "rb") as f:
            sha = hashlib.sha256(f.read()).hexdigest()
        with open(os.path.join(dev, "gps_frozen.py"), "w") as f:
            f.write('COMMIT = "abc1234"\n')     # "old": no SHA (first builds)
            if fz != "old":
                f.write('SHA = "%s"\n' % (sha if fz != "stale" else "0" * 64))
    env = dict(os.environ, PORT="/dev/fakeACM0", FAKE_DEV=dev,
               FAKE_LOG=os.path.join(work, "log"),
               FAKE_SHIM=os.path.join(work, ".venv", "bin", "shim.py"))
    env.update(env_extra)
    r = subprocess.run(["bash", os.path.join(work, "deploy.sh")], env=env,
                       capture_output=True, text=True, timeout=60)
    try:
        with open(env["FAKE_LOG"]) as f:
            calls = [json.loads(line) for line in f]
    except OSError:
        calls = []
    return r.returncode, r.stdout, r.stderr, dev, calls


def dev_file(dev, n):
    try:
        with open(os.path.join(dev, n), "rb") as f:
            return f.read()
    except OSError:
        return None


def gunzip(b):
    import gzip
    try:
        return gzip.decompress(b)
    except Exception:
        return None


if shutil.which("bash") and shutil.which("sha256sum"):
    with open(os.path.join(ROOT, "gps_logger.py"), "rb") as f:
        new_mpy = b"MPY" + f.read()
    with open(os.path.join(ROOT, "web.html"), "rb") as f:
        new_page = f.read()
    rc, out, err, dev, calls = deploy_run({})
    cps = [c for call in calls for c in call if c[0] == "cp"]
    check("DEPLOY: copies to .new, checks, renames; old .new and stray .py go",
          rc == 0 and dev_file(dev, "gps_logger.mpy") == new_mpy
          and dev_file(dev, "web.html") == new_page
          and dev_file(dev, "gps_logger.py") is None
          and not [n for n in os.listdir(dev) if n.endswith(".new")]
          and dev_file(dev, ".rtc") == b"0"
          and len(cps) == 4 and all(c[2].endswith(".new") for c in cps)
          and "Board: Generic ESP32C6" in out and "(rv32imc)" in out
          and "Deployed to /dev/fakeACM0" in out,
          "rc=%d %r %r %r" % (rc, out[-300:], err[-300:], cps))
    check("DEPLOY-GZIP: web.html.gz (gzip -9 -n of web.html) goes up too",
          gunzip(dev_file(dev, "web.html.gz") or b"") == new_page
          and (dev_file(dev, "web.html.gz") or b"")[4:8] == b"\0\0\0\0"
          and ["cp", "web.html.gz", ":web.html.gz.new"] in cps, repr(cps))
    rc, out, err, dev, calls = deploy_run({"FAKE_FREE_BLOCKS": "3"})
    check("DEPLOY: too little free flash -> stops before copying, old kept",
          rc != 0 and dev_file(dev, "gps_logger.mpy") == b"OLD MODULE"
          and not [c for call in calls for c in call if c[0] == "cp"]
          and "Not enough free flash" in err, "rc=%d %r" % (rc, err[-300:]))
    rc, out, err, dev, calls = deploy_run({"FAKE_TRUNC": "gps_logger.mpy"})
    check("DEPLOY: a cut copy fails the check -> old files kept, .new removed",
          rc != 0 and dev_file(dev, "gps_logger.mpy") == b"OLD MODULE"
          and dev_file(dev, "main.py") == b"old main"
          and not [n for n in os.listdir(dev) if n.endswith(".new")]
          and "Copy check FAILED" in err
          and "Deployed" not in out, "rc=%d %r" % (rc, err[-300:]))
    rc, out, err, dev, calls = deploy_run({"FAKE_FROZEN": "match"})
    cps = [c for call in calls for c in call if c[0] == "cp"]
    check("DEPLOY: frozen firmware -> main.py and the page only, the .mpy goes",
          rc == 0 and dev_file(dev, "gps_logger.mpy") is None
          and dev_file(dev, "gps_logger.py") is None
          and dev_file(dev, "web.html") == new_page
          and gunzip(dev_file(dev, "web.html.gz") or b"") == new_page
          and sorted(c[1] for c in cps) == ["main.py", "web.html",
                                            "web.html.gz"]
          and "frozen in (abc1234)" in out
          and "Deployed to /dev/fakeACM0" in out,
          "rc=%d %r %r %r" % (rc, out[-300:], err[-300:], cps))
    # DEPLOY-STALE: the frozen logger is not this gps_logger.py (or a first
    # build without SHA): stop before copying, unless FORCE=1
    stale = []
    for fz in ("stale", "old"):
        rc, out, err, dev, calls = deploy_run({"FAKE_FROZEN": fz})
        stale.append(rc != 0 and "tools/build_firmware.sh --flash" in err
                     and not [c for call in calls for c in call
                              if c[0] == "cp"]
                     and dev_file(dev, "main.py") == b"old main"
                     and dev_file(dev, "gps_logger.mpy") == b"OLD MODULE")
    rc, out, err, dev, calls = deploy_run({"FAKE_FROZEN": "stale",
                                           "FORCE": "1"})
    stale.append(rc == 0 and dev_file(dev, "main.py") != b"old main"
                 and dev_file(dev, "gps_logger.mpy") is None
                 and "FORCE=1" in err and "Deployed" in out)
    check("DEPLOY-STALE: frozen logger != gps_logger.py (or no SHA) stops; "
          "FORCE=1 deploys", stale == [True, True, True],
          "%r rc=%d %r %r" % (stale, rc, out[-200:], err[-300:]))
    # DEPLOY-UNSURE: no clear frozen/plain answer -> stop, nothing installed
    rc, out, err, dev, calls = deploy_run({"FAKE_GARBLE": "1"})
    check("DEPLOY-UNSURE: no frozen/plain answer -> exit 1, no .mpy built or "
          "copied", rc != 0 and "Could not tell" in err
          and not [c for call in calls for c in call if c[0] == "cp"]
          and dev_file(dev, "gps_logger.mpy") == b"OLD MODULE"
          and not os.path.exists(os.path.join(os.path.dirname(dev),
                                              "gps_logger.mpy")),
          "rc=%d %r %r" % (rc, out[-200:], err[-200:]))
    # DEPLOY-FORCE-MPY: the emergency path installs the .mpy over the frozen
    rc, out, err, dev, calls = deploy_run({"FAKE_FROZEN": "stale",
                                           "FORCE_MPY": "1"})
    check("DEPLOY-FORCE-MPY: FORCE_MPY=1 installs gps_logger.mpy on frozen "
          "firmware, loudly", rc == 0 and dev_file(dev, "gps_logger.mpy")
          == new_mpy and dev_file(dev, "gps_logger.py") is None
          and "WARNING: FORCE_MPY=1" in out and "Deployed" in out,
          "rc=%d %r %r" % (rc, out[-300:], err[-200:]))
else:
    print("SKIP  DEPLOY: no bash/sha256sum")

# BUILD: tools/build_firmware.sh with a fake ESP-IDF, MicroPython tree,
# esptool and mpremote: the marker module records the SHA-256 of the frozen
# gps_logger.py and a -dirty commit; --flash touches nothing on the board
# before (or instead of) a successful write (it removed gps_logger.mpy
# first: a failed check left a stock board without a logger)
FAKE_ESPTOOL = r"""#!/usr/bin/env python3
import os, sys, json
a = sys.argv[1:]
with open(os.environ["FAKE_LOG"], "a") as f:
    f.write(json.dumps(["esptool"] + a) + "\n")
if "read_flash" in a:
    i = a.index("read_flash")
    with open(a[i + 3], "wb") as f:
        f.write(os.environ.get("FAKE_PT", "PT").encode()
                if a[i + 1] == "0x8000" else b"APP")
if "write_flash" in a and os.environ.get("FAKE_WRITE_FAIL"):
    sys.exit(2)
"""
FAKE_BUILD_MPREMOTE = r"""#!/usr/bin/env python3
import os, sys, json
with open(os.environ["FAKE_LOG"], "a") as f:
    f.write(json.dumps(["mpremote"] + sys.argv[1:]) + "\n")
print("board ready")
"""
FAKE_PORT_MK = ("all:\n\tmkdir -p build-$(BOARD)/partition_table\n"
                "\tprintf BIN > build-$(BOARD)/micropython.bin\n"
                "\tprintf PT > build-$(BOARD)/partition_table/"
                "partition-table.bin\n")


def build_run(args, env_extra, dirty=False):
    """tools/build_firmware.sh in a fresh git repo with fakes:
    (rc, stdout, stderr, work dir, calls)."""
    work = tempfile.mkdtemp(dir=dp_td)
    os.makedirs(os.path.join(work, "tools"))
    shutil.copy(os.path.join(ROOT, "tools", "build_firmware.sh"),
                os.path.join(work, "tools"))
    shutil.copy(os.path.join(ROOT, "gps_logger.py"), work)
    fake = os.path.join(work, "fake")
    for d in ("bin", "idf", "mpy/mpy-cross", "mpy/ports/esp32"):
        os.makedirs(os.path.join(fake, d))
    os.makedirs(os.path.join(work, ".venv", "bin"))
    for path, src in (("fake/bin/esptool.py", FAKE_ESPTOOL),
                      (".venv/bin/mpremote", FAKE_BUILD_MPREMOTE),
                      ("fake/idf/export.sh", ""),
                      ("fake/mpy/mpy-cross/Makefile", "all:\n\t@true\n"),
                      ("fake/mpy/ports/esp32/Makefile", FAKE_PORT_MK)):
        with open(os.path.join(work, path), "w") as f:
            f.write(src)
        os.chmod(os.path.join(work, path), 0o755)
    git = ["git", "-C", work, "-c", "user.name=t", "-c", "user.email=t@t"]
    subprocess.run(git + ["init", "-q"], check=True)
    subprocess.run(git + ["add", "gps_logger.py"], check=True)
    subprocess.run(git + ["commit", "-q", "-m", "x"], check=True)
    if dirty:
        with open(os.path.join(work, "gps_logger.py"), "a") as f:
            f.write("# changed\n")
    env = dict(os.environ, IDF_DIR=os.path.join(fake, "idf"),
               MPY_DIR=os.path.join(fake, "mpy"), PORT="/dev/fakeACM0",
               FAKE_LOG=os.path.join(work, "log"),
               PATH=os.path.join(fake, "bin") + os.pathsep
               + os.environ.get("PATH", ""))
    env.update(env_extra)
    r = subprocess.run(["bash", os.path.join(work, "tools",
                                             "build_firmware.sh")] + args,
                       env=env, capture_output=True, text=True, timeout=120)
    try:
        with open(env["FAKE_LOG"]) as f:
            calls = [json.loads(line) for line in f]
    except OSError:
        calls = []
    return r.returncode, r.stdout, r.stderr, work, calls


def frozen_marker(work):
    m = {}
    try:
        with open(os.path.join(work, "firmware", "gps_frozen.py")) as f:
            exec(f.read(), m)
    except (OSError, SyntaxError):
        pass
    return m


if all(shutil.which(t) for t in ("bash", "sha256sum", "git", "make")):
    marks = []
    for dirty in (False, True):
        rc, out, err, work, calls = build_run([], {}, dirty=dirty)
        with open(os.path.join(work, "gps_logger.py"), "rb") as f:
            sha = hashlib.sha256(f.read()).hexdigest()
        head = subprocess.run(["git", "-C", work, "rev-parse", "--short",
                               "HEAD"], capture_output=True,
                              text=True).stdout.strip()
        m = frozen_marker(work)
        marks.append(rc == 0 and m.get("SHA") == sha and m.get("COMMIT")
                     == head + ("-dirty" if dirty else "")
                     and not calls)
    check("BUILD-MARKER: gps_frozen has the SHA-256 of the frozen "
          "gps_logger.py, COMMIT -dirty with changes", marks == [True, True],
          "%r rc=%d %r %r" % (marks, rc, out[-200:], err[-300:]))
    rc, out, err, work, calls = build_run(["--flash"], {"FAKE_PT": "OTHER"})
    check("BUILD-FLASH: a partition table mismatch stops before anything "
          "on the board is touched",
          rc != 0 and "partition table differs" in err
          and not [c for c in calls if c[0] == "mpremote"]
          and not [c for c in calls if "write_flash" in c],
          "rc=%d %r %r" % (rc, calls, err[-200:]))
    rc, out, err, work, calls = build_run(["--flash"],
                                          {"FAKE_WRITE_FAIL": "1"})
    check("BUILD-FLASH: a failed write leaves the board's files alone",
          rc != 0 and not [c for c in calls if c[0] == "mpremote"]
          and [c for c in calls if "write_flash" in c],
          "rc=%d %r" % (rc, calls))
    rc, out, err, work, calls = build_run(["--flash"], {})
    steps = [c[c.index("read_flash") + 1] if "read_flash" in c else
             "write" if "write_flash" in c else c[0] for c in calls]
    check("BUILD-FLASH: check, backup, write; no REPL step; deploy.sh named "
          "for the cleanup and after a rollback",
          rc == 0 and steps == ["0x8000", "0x10000", "write"]
          and "./deploy.sh" in out
          and "write_flash 0x10000" in out
          and out.rstrip().endswith(", then ./deploy.sh"),
          "rc=%d %r %r %r" % (rc, steps, out[-300:], err[-200:]))
else:
    print("SKIP  BUILD: no bash/sha256sum/git/make")
shutil.rmtree(dp_td, ignore_errors=True)


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
    gl._up[:] = [0, 0]                   # uptime from tick 0
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
    rows = dict(gl._web_rows())
    check("UPTIME: the JSON keeps seconds and gives the previous run as text",
          isinstance(rows["uptime"], int) and rows["prev_uptime"] == 3494
          and rows["prev_run"] == "58 min 14 s (last line "
                                  "2026-10-28 12:59:42Z)", repr(rows))

    import inspect
    loop_src = inspect.getsource(gl._loop)
    check("DIAG: _loop writes a line every DIAG ms",
          "_diag(now)" in loop_src and "DIAG" in loop_src)
finally:
    gl.DIAG, gl.DIAG_MAX = saved_diag


# REV29: fixes from the 2026-10-29 review (one check per finding)
def rv_nmea(body):
    c = 0
    for ch in body.encode():
        c ^= ch
    return ("$%s*%02X" % (body, c)).encode()


# GPS-2: GSA every GSA_EVERY s; the page stops waiting for a silent list
gl.reset()
gl.st["setup_ok"] = True
_uart_rx[:] = []
_uart_tx[:] = []
gl.sv_poll(_clock[0])
_clock[0] += gl.SV_POLL
gl.sv_poll(_clock[0])
_clock[0] += gl.SV_POLL
gl.sv_poll(_clock[0])
miss = gl.st["sv_miss"]
t0 = _clock[0] + gl.SV_FRESH
_clock[0] = t0
gl._sv_fresh()
check("GPS-2: GSA rate is one round per GSA_EVERY s at GPS_HZ",
      (0x02, gl.GPS_HZ * gl.GSA_EVERY) in gl.GPS_MSGS)
check("GPS-2: unanswered polls are counted; the page then does not wait",
      miss == 2 and _clock[0] - t0 < 100, "miss=%d waited=%d"
      % (miss, _clock[0] - t0))

# GPS-7: a lost ACK leaves setup_ok False with nothing extra: set up again
gl.reset()
_uart_drop[0] = 99                       # every ACK lost
try:
    gl.gps_setup()
finally:
    _uart_drop[0] = 0
lost = gl.st["setup_ok"]
_uart_tx[:] = []
_clock[0] += gl.GPS_RETRY + 1000
gl.gps_check(_clock[0])
again = len([f for f in _uart_tx if f[2:3] == b"\x06"]) > 0
check("GPS-7: setup not acknowledged and no extra sentence -> sent again",
      lost is False and gl.st["extra"] == 0 and again
      and gl.st["setup_ok"] is True, "%r %r" % (lost, gl.st["setup_ok"]))

# GPS-18: a GSA flood (rate not taken) counts as a setup not taken
gl.reset()
gl.st["setup_at"] = _clock[0] - 5000
for _ in range(20):
    gl.handle(rv_nmea("GNGSA,A,3,01,02,03,,,,,,,,,,1.5,0.9,1.2,1"))
flood = gl.st["extra"]
gl.reset()
gl.st["setup_at"] = _clock[0] - 5000
for _ in range(4):
    gl.handle(rv_nmea("GNGSA,A,3,01,02,03,,,,,,,,,,1.5,0.9,1.2,1"))
check("GPS-18: >GSA_MAX GSA a second -> extra; one round of 4 does not",
      flood >= 1 and gl.st["extra"] == 0, "%d %d" % (flood, gl.st["extra"]))

# SETUP-LATE: an overrun receiver that ACKs every frame 1.5 s late blocked
# the loop 10-30 s and the NMEA read meanwhile was dropped (its start-up
# banner too). Now GPS_SETUP_MAX at most, the rest goes out once, and what
# is read meanwhile goes through the parser
late = []
real_write, real_any = UART.write, UART.any


def late_write(self, data):
    data = bytes(data)
    _uart_tx.append(data)
    if data[:3] == b"\xb5\x62\x06":
        late.append((_clock[0] + 1500, gl._ubx(0x05, 0x01, data[2:4])))
    return len(data)


def late_any(self):
    late.sort()
    while late and late[0][0] <= _clock[0]:
        _uart_rx.extend(late.pop(0)[1])
    return len(_uart_rx)


setup_max = getattr(gl, "GPS_SETUP_MAX", 4000)
gl.reset()
_uart_rx[:] = []
_uart_tx[:] = []
late.append((_clock[0] + 500, rv_nmea(
    "GPTXT,01,01,02,u-blox ag - www.u-blox.com") + b"\r\n"))
late.append((_clock[0] + 700, rv_nmea(
    "GPRMC,120500.00,V,,,,,,,171026,,,N") + b"\r\n"))
UART.write, UART.any = late_write, late_any
t0 = _clock[0]
try:
    gl.gps_setup()
finally:
    UART.write, UART.any = real_write, real_any
    late[:] = []
took = _clock[0] - t0
sent = [f.hex() for f in _uart_tx]
msgs = [gl._ubx(0x06, 0x01, bytes((0xF0, m, r))).hex() for m, r in gl.GPS_MSGS]
check("SETUP-LATE: late ACKs block GPS_SETUP_MAX at most, the rest goes once",
      took <= setup_max + 400 and gl.st["setup_ok"] is False
      and all(m in sent for m in msgs)
      and gl._rate(1000 // gl.GPS_HZ).hex() not in sent,
      "ms=%d ok=%r frames=%d" % (took, gl.st["setup_ok"], len(sent)))
check("SETUP-LATE: NMEA read during the setup is parsed (banner = restart)",
      gl.st["gps_boots"] == 1 and gl.st["ok"] >= 2 and gl.st["extra"] == 1,
      "boots=%d ok=%d extra=%d" % (gl.st["gps_boots"], gl.st["ok"],
                                   gl.st["extra"]))
_uart_rx[:] = []

# SETUP-CHECK: an acknowledged setup that did not take (still one fix a
# second, or no GST: GPS_MAX_STD silently off) is sent again after
# GPS_VERIFY; one that took (GPS_HZ fixes and GST) is left alone


def verify_run(fixes, gst):
    gl.reset()
    _uart_rx[:] = []
    gl.gps_setup()
    _uart_tx[:] = []
    for k in range(12):
        for f in range(fixes):
            t = "1206%02d.%02d" % (k, f * 20)
            gl.handle(rv_nmea("GPGGA,%s,4730.000,N,01900.000,E,1,08,1.0,"
                              "100.0,M,40.0,M,," % t))
            gl.handle(rv_nmea("GPRMC,%s,A,4730.000,N,01900.000,E,0.1,90.0,"
                              "171026,,,A" % t))
        if gst:
            gl.handle(rv_nmea("GPGST,1206%02d.00,20,,,,3.0,2.5,6.0" % k))
        _clock[0] += 1000
        gl.gps_check(_clock[0])
    return len([f for f in _uart_tx if f[2:3] == b"\x06"])


v_slow = verify_run(1, True)
v_good = verify_run(gl.GPS_HZ, True)
v_nogst = verify_run(gl.GPS_HZ, False)
check("SETUP-CHECK: acknowledged but still 1 fix/s, or no GST -> set up again",
      v_slow > 0 and v_nogst > 0 and v_good == 0,
      "slow=%d nogst=%d good=%d" % (v_slow, v_nogst, v_good))

# SETUP-NOFIX: a setup before the first fix (at boot, indoors) got its one
# verdict GPS_VERIFY later with no valid second to tell by: "taken", the
# backoff reset, and a receiver left at 1 Hz (a stale RATE ACK) was never
# set up again. Now no verdict before 3 valid seconds (up to
# GPS_VERIFY_MAX), and taken only with GPS_HZ fixes and GST


def nofix_setup():
    gl.reset()
    _uart_rx[:] = []
    gl.gps_setup()
    gl.st["ack_n"] = 2                   # a backoff that is running
    _uart_tx[:] = []


def nofix_secs(k0, n, fix=False, gst_every=0, step=1):
    for k in range(k0, k0 + n, step):
        t = "12%02d%02d.00" % (k // 60 % 60, k % 60)
        if fix:
            gl.handle(rv_nmea("GPGGA,%s,4730.000,N,01900.000,E,1,08,1.0,"
                              "100.0,M,40.0,M,," % t))
            gl.handle(rv_nmea("GPRMC,%s,A,4730.000,N,01900.000,E,0.1,90.0,"
                              "171026,,,A" % t))
        else:
            gl.handle(rv_nmea("GPRMC,%s,V,,,,,,,171026,,,N" % t))
            gl.handle(rv_nmea("GPGGA,%s,,,,,0,00,99.99,,,,,," % t))
        if gst_every and k % gst_every == 0:
            gl.handle(rv_nmea("GPGST,%s,,,,,,," % t))
        _clock[0] += 1000 * step
        gl.gps_check(_clock[0])


def cfgs():
    return len([f for f in _uart_tx if f[2:3] == b"\x06"])


nofix_setup()
nofix_secs(0, 12, gst_every=5)
nf_wait = (gl.st["checked"], gl.st["ack_n"], cfgs())
nofix_secs(12, 30, fix=True)
nf_again = (cfgs(), gl.st["ack_n"])
check("SETUP-NOFIX: no verdict before a fix, then 1 fix/s -> set up again",
      nf_wait == (False, 2, 0) and nf_again[0] > 0 and nf_again[1] >= 2,
      "wait=%r again=%r" % (nf_wait, nf_again))
nofix_setup()
nofix_secs(0, getattr(gl, "GPS_VERIFY_MAX", 1800000) // 1000 + 20,
           gst_every=5, step=10)
check("SETUP-NOFIX: no fix for GPS_VERIFY_MAX -> the wait ends, no verdict",
      gl.st["checked"] is True and gl.st["ack_n"] == 2 and cfgs() == 0
      and not gl.st["extra"], "%r %r %d" % (gl.st["checked"],
                                          gl.st["ack_n"], cfgs()))

# NO-GST-NOFIX: an M10 without a fix (indoors) may send no GST: that was a
# "no GST" verdict, so indoor stretches were set up again and again
nofix_setup()
nofix_secs(0, 70)
check("NO-GST-NOFIX: no fix and no GST -> no verdict, not set up again",
      cfgs() == 0 and gl.st["checked"] is False and not gl.st["extra"],
      "cfg=%d checked=%r extra=%r" % (cfgs(), gl.st["checked"],
                                      gl.st["extra"]))

# SETUP-NOT-TAKEN: a session left at 1 Hz was silent in diag.log; now a
# line per GPS_NOT_TAKEN_SAY, the verdicts between counted
saved_d = gl.DIAG
gl.DIAG = 60000
try:
    os.remove(gl.DIAG_FILE)
except OSError:
    pass
gl.reset()
for k in range(12):                      # a verdict every 70 s
    gl.st.update(checked=False, setup_at=_clock[0] - gl.GPS_VERIFY, hz1=3,
                 hz5=0)
    gl._setup_check(_clock[0])
    _clock[0] += 70000
try:
    with open(gl.DIAG_FILE) as f:
        nt = [ln for ln in f.read().splitlines()
              if ln.startswith("gps setup not taken")]
except OSError:
    nt = []
gl.DIAG = saved_d
check("SETUP-NOT-TAKEN: a diag line per GPS_NOT_TAKEN_SAY, the rest counted",
      nt == ["gps setup not taken: 1 fix/s",
             "gps setup not taken: 1 fix/s (8 more)"], repr(nt))

# SETUP-BACKOFF: a receiver that acknowledges every setup but keeps sending
# the sentences it turns off: GPS_RETRY, then doubling up to a cap
gl.reset()
_uart_rx[:] = []
gl.gps_setup()
times = []
for _ in range(200):
    _clock[0] += 1500
    gl.handle(rv_nmea("GPVTG,,T,,M,0.0,N,0.0,K,A"))
    n0 = len(_uart_tx)
    gl.gps_check(_clock[0])
    if any(f[2:3] == b"\x06" for f in _uart_tx[n0:]):
        times.append(_clock[0])
gaps = [(b2 - a2) // 1000 for a2, b2 in zip(times, times[1:])]
cap = getattr(gl, "GPS_IGNORED_MAX", 60000) // 1000
check("SETUP-BACKOFF: acknowledged but ignored -> retries double to a cap",
      len(gaps) >= 4 and gaps[0] <= gl.GPS_RETRY // 1000 + 2
      and gaps == sorted(gaps) and cap <= gaps[-1] <= cap + 2,
      "gaps=%s" % gaps)
gl.handle(rv_nmea("GPTXT,01,01,02,u-blox ag - www.u-blox.com"))
check("SETUP-BACKOFF: a receiver restart starts the backoff afresh",
      gl.st.get("ack_n") == 0 and gl.st["setup_wait"] == gl.GPS_RETRY,
      "%r %r" % (gl.st.get("ack_n"), gl.st["setup_wait"]))

# SATS-ORDER: a satellite poll and a setup due in the same gps_check: the
# setup first, the poll right after it (before, its answer was lost in the
# setup and counted as a miss)
gl.reset()
_uart_rx[:] = []
_uart_tx[:] = []
pl = bytearray(8)
pl[5] = 1
pl += bytes((0, 7, 41, 60, 10, 0, 0, 0, 0x1F, 0, 0, 0))
_uart_sat[0] = gl._ubx(0x01, 0x35, bytes(pl))
try:
    gl.st["setup_at"] = _clock[0] - 60000      # not acknowledged: due
    gl.gps_check(_clock[0])
    gl.read_uart()
finally:
    _uart_sat[0] = None
cfg = [k for k, f in enumerate(_uart_tx) if f[2:3] == b"\x06"]
poll = [k for k, f in enumerate(_uart_tx) if f[:4] == b"\xb5\x62\x01\x35"]
check("SATS-ORDER: the poll goes after a setup due at the same time",
      cfg and poll and poll[0] > cfg[-1] and gl.st["svs"] is not None
      and gl.st["sv_miss"] == 0, "cfg=%r poll=%r" % (cfg[-1:], poll[:1]))

# GPS-19: the rotation's first line is no status line: look in diag.1
diag_clear()
with open(gl.DIAG_OLD, "w") as f:
    f.write("boot cause 1 prev_up - mhz 80 mem 1\n"
            "up 3494 utc 20261028T125942Z nmea 1/0\n")
with open(gl.DIAG_FILE, "w") as f:
    f.write("gps u-blox M10, ROM SPG 5.10 (7b202e), PROTVER=34.10\n")
gl._diag_prev()
check("GPS-19: previous run found in diag.1 behind a non-status line",
      gl.prev_run["up"] == 3494 and gl.prev_run["how"] == "status",
      repr(gl.prev_run))
diag_clear()

# DIAG-TAIL: status lines are ~180 B, so 160 B a line gave the page 13 of
# WEB_DIAG lines, and right after a rotation only one; now the page takes
# the rest from diag.1, and the tail is read in small pieces (one big read
# needs a free block that big)
def st_line(k):
    return ("up %d utc 20261031T0530%02dZ nmea 99999/12 gpsboots 0 loops 0 "
            "sats 21/13 fix 3D_fix hdop 0.88199996 points 40000 sent 39990 "
            "wifi connected pulses 0 mem 27000 idf 157/20 heap 124" % (
                60 * k, k % 60))


diag_clear()
with open(gl.DIAG_OLD, "w") as f:
    f.write("".join(st_line(k) + "\n" for k in range(40)))
with open(gl.DIAG_FILE, "w") as f:
    f.write("".join(st_line(k) + "\n" for k in range(40, 80)))
reads = []
real_open = open


class ReadLog(object):
    def __init__(self, f):
        self.f = f

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.f.close()

    def read(self, n=-1):
        reads.append(n)
        return self.f.read(n)

    def __getattr__(self, k):
        return getattr(self.f, k)


gl.open = lambda *a, **k: ReadLog(real_open(*a, **k))
try:
    tail = gl._diag_tail(gl.DIAG_FILE, gl.WEB_DIAG)
finally:
    del gl.open
with open(gl.DIAG_FILE, "w") as f:
    f.write("gps u-blox M10, ROM SPG 5.10 (7b202e), PROTVER=34.10\n")
page = gl._web_diag()
check("DIAG-TAIL: WEB_DIAG lines of ~180 B, read in small pieces",
      len(st_line(79)) > 175 and tail == [st_line(k) for k in range(65, 80)]
      and reads and all(0 < n <= 1024 for n in reads),
      "%d lines, reads %r" % (len(tail), reads))
check("DIAG-TAIL: just after a rotation the page fills up from diag.1",
      len(page) == gl.WEB_DIAG and page[-1].startswith("gps u-blox M10")
      and page[0] == st_line(26) and page[-2] == st_line(39),
      "%d %r" % (len(page), page[:1]))

# DIAG-TAIL-EDGE: a window that starts right at a line start dropped that
# whole line as a partial one (lines of DIAG_LINE - 1 B: 7 of 8); every
# line wholly in the window is kept


def tail_ref(data, n):
    start = max(len(data) - gl.DIAG_LINE * n, 0)
    out, off = [], 0
    for ln in data.split(b"\n"):
        if off >= start and ln:
            try:
                out.append(ln.decode())
            except UnicodeError:
                pass
        off += len(ln) + 1
    return out[-n:]


with open(gl.DIAG_FILE, "w") as f:
    f.write("".join("%03d" % k + "x" * (gl.DIAG_LINE - 4) + "\n"
                    for k in range(20)))
edge = gl._diag_tail(gl.DIAG_FILE, 8)
import random
t_rnd = random.Random(5)
t_bad = 0
for _ in range(300):
    t_data = b"\n".join(
        bytes(t_rnd.choice(b"ab \xff") for _ in range(t_rnd.choice(
            (0, 1, 50, 179, 219, 220, 221, 511, 512, 513))))
        for _ in range(t_rnd.randrange(0, 30)))
    if t_rnd.random() < 0.8:
        t_data += b"\n"
    with open(gl.DIAG_FILE, "wb") as f:
        f.write(t_data)
    t_n = t_rnd.choice((1, 8, 15))
    if gl._diag_tail(gl.DIAG_FILE, t_n) != tail_ref(t_data, t_n):
        t_bad += 1
check("DIAG-TAIL-EDGE: a line starting right at the window is not dropped",
      len(edge) == 8 and edge[0].startswith("012") and t_bad == 0,
      "%d lines, first %r, %d of 300 random files differ" % (
          len(edge), edge[:1] and edge[0][:3], t_bad))
diag_clear()

# GPS-20: a cut-off sentence glued to a whole one keeps the whole one
gl.reset()
gl.handle(b"$GNGGA,120000.80,4728.0" + rv_nmea(
    "GNRMC,120001.00,A,4728.0000,N,01900.0000,E,0.1,,291026,,,A,V"))
check("GPS-20: the whole sentence after a cut-off one is parsed",
      gl.st["ok"] == 1 and gl.st["err"] == 0 and gl.st["status"] == "A",
      "%d %d" % (gl.st["ok"], gl.st["err"]))

# GPS-21: uptime sums tick differences across a ticks_ms wrap
TPW = 1 << 30
saved_t = (time.ticks_ms, time.ticks_diff)
time.ticks_diff = lambda a, b: ((a - b + TPW // 2) & (TPW - 1)) - TPW // 2
try:
    gl._up[:] = [0, TPW - 1000]
    ups = []
    for k in range(1, 15):
        ups.append(gl.uptime_s((TPW - 1000 + k * 86400000) & (TPW - 1)))
finally:
    time.ticks_ms, time.ticks_diff = saved_t
check("GPS-21: uptime keeps counting past the 12.4-day ticks wrap",
      ups == [k * 86400 for k in range(1, 15)], repr(ups[-3:]))

# GPS: used from the satellite list (GGA stops at 12), banner split
gl.reset()
gl.st["used"] = 12
gl.st["svs"] = [(k, 30, 40, 100, True, False, 0) for k in range(1, 17)]
gl.st["svs_at"] = _clock[0]
used_list = gl.sv_used()
gl.st["svs_at"] = _clock[0] - 3 * gl.SV_POLL
used_old = gl.sv_used()
check("GPS: used from a fresh satellite list, else GGA",
      used_list == 16 and used_old == 12, "%r %r" % (used_list, used_old))
gl.reset()
_uart_rx[:] = []
ban = rv_nmea("GPTXT,01,01,02,u-blox ag - www.u-blox.com") + b"\r\n"
_uart_rx.extend(b"x" * 10)               # a cut-off line before it
gl.read_uart()
_uart_rx.extend(ban[:12])
gl.read_uart()
_uart_rx.extend(ban[12:])
gl.read_uart()
check("GPS: a start-up banner split across two reads is counted",
      gl.st["banners"] == 1, gl.st["banners"])

# UBX-JUNK: sync bytes in junk with a plausible length are no frame: the
# bad checksum skips only them, and the NMEA they would have swallowed
# (up to ~1 KB) is read
gl.reset()
_uart_rx[:] = []
rmcs = b"".join(rv_nmea("GPRMC,1207%02d.00,V,,,,,,,171026,,,N" % k) + b"\r\n"
                for k in range(3))
_uart_rx.extend(b"\xff\xb5\x62\x01\x02\x40\x00" + rmcs)
gl.read_uart()
check("UBX-JUNK: a false UBX sync costs one error, not the NMEA behind it",
      gl.st["ok"] == 3 and gl.st["err"] == 1,
      "ok=%d err=%d" % (gl.st["ok"], gl.st["err"]))

# RXBUF: lines and frames are read in place from one preallocated buffer
# (a bytes buffer grown per read and sliced per line copied ~1 KB at a
# time); a frame split over reads, a line after it and junk still work
gl.reset()
_uart_rx[:] = []
frame = gl._ubx(0x01, 0x35, bytes(pl))
data = rmcs + frame + rmcs
for k in range(0, len(data), 7):
    _uart_rx.extend(data[k:k + 7])
    gl.read_uart()
rx_ok = (gl.st["ok"], gl.st["err"], gl.st["svs"] is not None)
_uart_rx.extend(b"\x00" * 300)               # junk: no newline in sight
gl.read_uart()
_uart_rx.extend(rmcs)
gl.read_uart()
check("RXBUF: a preallocated buffer, frames and lines across 7-byte reads",
      isinstance(getattr(gl, "_rx", None), bytearray)
      and rx_ok == (6, 0, True) and gl.st["ok"] == 9,
      "%r ok=%d" % (rx_ok, gl.st["ok"]))

# RXBUF-BANNER: a reset loop's banner start cut off right at the RX_LINE
# junk reset ("$GPTXT,01,01,02," last, its "u" in the next read) went with
# the junk, and the reset loops came up short (125 of 3000 fuzz runs); a
# banner start at the end of the junk stays now
gl.reset()
_uart_rx[:] = []
_uart_rx.extend(b"\xff" * (gl.RX_LINE + 40 - 16) + b"$GPTXT,01,01,02,")
gl.read_uart()
_uart_rx.extend(b"u" + b"\xff" * 30)
gl.read_uart()
cut_ban = gl.st["banners"]
import random
b_rnd = random.Random(3)
b_bad = 0
for _ in range(400):
    gl.reset()
    _uart_rx[:] = []
    b_data, b_n = b"", 0
    for _ in range(b_rnd.randrange(1, 30)):
        if b_rnd.random() < 0.4:
            b_data += b"$GPTXT,01,01,02,u" + b"\xff" * b_rnd.randrange(400)
            b_n += 1
        else:
            b_data += rv_nmea("GNRMC,120000.00,V,,,,,,,311026,,,N") + b"\r\n"
    k = 0
    while k < len(b_data):
        step = b_rnd.choice((1, 3, 13, 14, 64, 256, 300))
        _uart_rx.extend(b_data[k:k + step])
        k += step
        if b_rnd.random() < 0.5:
            gl.read_uart()
    gl.read_uart()
    if gl.st["banners"] != b_n:
        b_bad += 1
check("RXBUF-BANNER: a banner start cut off by the junk reset is counted",
      cut_ban == 1 and b_bad == 0,
      "cut=%d, %d of 400 random streams miscounted" % (cut_ban, b_bad))


# RXBUF-FIND: line ends, UBX sync and banners are found with bytearray.find
# (C), not a Python loop over every byte (~0.17 us/B on the unix port)
class IdxCount(bytearray):
    n = 0

    def __getitem__(self, k):
        IdxCount.n += 1
        return bytearray.__getitem__(self, k)


saved_rx = (gl._rx, gl._rxv)
ic = IdxCount(len(gl._rx))
gl._rx, gl._rxv = ic, memoryview(ic)
gl._rx_n = gl._rx_s = 0
try:
    gl.reset()
    _uart_rx[:] = []
    f_data = (b"".join(rv_nmea("GPRMC,1210%02d.00,V,,,,,,,171026,,,N" % k)
                       + b"\r\n" for k in range(40))
              + (rv_nmea("GPTXT,01,01,02,u-blox ag - www.u-blox.com")
                 + b"\r\n") * 2)
    _uart_rx.extend(f_data)
    gl.read_uart()
finally:
    gl._rx, gl._rxv = saved_rx
    gl._rx_n = gl._rx_s = 0
check("RXBUF-FIND: no per-byte Python loop over what the UART delivers",
      IdxCount.n < len(f_data) // 20 and gl.st["ok"] == 42
      and gl.st["banners"] == 2,
      "%d indexings for %d B, ok=%d banners=%d" % (
          IdxCount.n, len(f_data), gl.st["ok"], gl.st["banners"]))


# RXBUF-DROP: the rest moves to the front in one slice store (MicroPython
# copies it with memmove), not in non-overlapping pieces
class SetCount(object):
    def __init__(self, mv):
        self.mv, self.sets = mv, 0

    def __getitem__(self, k):
        return self.mv[k]

    def __setitem__(self, k, v):
        self.sets += 1
        self.mv[k] = v


saved_rxv = gl._rxv
gl._rx[:600] = bytes(range(256)) * 2 + bytes(range(88))
d_want = bytes(gl._rx[100:600])
sc = SetCount(memoryview(gl._rx))
gl._rxv, gl._rx_n = sc, 600
try:
    gl._rx_drop(100)
    dropped = (bytes(gl._rx[:500]) == d_want, gl._rx_n, sc.sets)
finally:
    gl._rxv = saved_rxv
    gl._rx_n = gl._rx_s = 0
check("RXBUF-DROP: the rest moves to the front in one slice store",
      dropped == (True, 500, 1), repr(dropped))


# PVT: UBX-NAV-PVT on a u-blox M8/M10 (new feature, 2026-10-31): parsing,
# the weighted integer mean, float-free track lines (#TRK2), the setup per
# receiver kind with its NMEA fallback, upload fields
import struct as _struct


def pvt_frame(ymdhms, lat, lon, hacc=1.5, fix=3, flags=0x01, flags3=0,
              valid=0x07, tacc=50, nano=0, hmsl=100.0, mps=0.0, head=0.0,
              vacc=2.5, nsv=12, pdop=1.2, sacc=0.3, hacc_mm=None):
    """A UBX-NAV-PVT frame (92-byte payload, u-blox M10 layout). lat/lon
    are degrees, or ints already in 1e-7 degrees."""
    y, mo, d, h, mi, s = ymdhms
    li = lat if isinstance(lat, int) else int(round(lat * 1e7))
    lo = lon if isinstance(lon, int) else int(round(lon * 1e7))
    p = _struct.pack("<IHBBBBBBIiBBBBiiiiIIiiiiiIIHH4xihH",
                     0, y, mo, d, h, mi, s, valid, tacc, nano, fix, flags,
                     0, nsv, lo, li, int(hmsl * 1000) + 40000,
                     int(round(hmsl * 1000)),
                     int(round(hacc * 1000)) if hacc_mm is None else hacc_mm,
                     int(round(vacc * 1000)), 0, 0, 0, int(round(mps * 1000)),
                     int(round(head * 100000)), int(round(sacc * 1000)),
                     500000, int(round(pdop * 100)), flags3, 0, 0, 0)
    return gl._ubx(0x01, 0x07, p)


def pvt_second(ymdhms, n=5, **kw):
    """n epochs of one second (0.2 s apart): kw values may be lists."""
    for k in range(n):
        a = dict((key, v[k] if isinstance(v, list) else v)
                 for key, v in kw.items())
        lat, lon = a.pop("lat", 47.5), a.pop("lon", 19.0)
        gl._ubx_frame(pvt_frame(ymdhms, lat, lon, nano=k * 200000000, **a))


def pvt_monver(hw, protver):
    pl = b"ROM SPG 5.10 (7b202e)".ljust(30, b"\0") + hw.ljust(10, b"\0")
    pl += protver.ljust(30, b"\0")
    return gl._ubx(0x0A, 0x04, pl)


T0 = (2026, 10, 31, 12, 0, 0)


def t_at(s, base=T0):
    return base[:5] + (base[5] + s,)


# PVT-PARSE: one unpack_from per frame fills the fields GGA/RMC fill
gl.reset()
pvt_second(T0, lat=-33.8567844, lon=-151.2152967, hacc=2.0, mps=1.5,
           head=271.5, hmsl=58.6, vacc=3.0, nsv=17, pdop=1.4, sacc=0.4)
pre = (gl.st["seq"], gl.st["valid"])
pvt_second(t_at(1), n=1)
check("PVT-PARSE: a second is published when the next one starts",
      pre == (0, False) and gl.st["seq"] == 1 and gl.st["valid"] is True
      and gl.st["time"] == "12:00:00" and gl.st["date"] == "2026-10-31"
      and gl.st["fix_src"] == "pvt", repr((pre, gl.st["time"])))
check("PVT-PARSE: negative lat/lon kept as 1e-7 ints, accuracy, elevation",
      gl.st["lat_i"] == -338567844 and gl.st["lon_i"] == -1512152967
      and abs(gl.st["lat"] + 33.8567844) < 1e-9
      and abs(gl.st["hacc"] - 2.0) < 1e-9 and abs(gl.st["alt"] - 58.6) < 1e-9
      and abs(gl.st["mps"] - 1.5) < 1e-9 and gl.st["epochs"] == 5,
      repr((gl.st["lat_i"], gl.st["lon_i"], gl.st["hacc"], gl.st["alt"])))
pvt_second(T0, n=1, lat=47.5, lon=19.0, mps=2.0, head=12.5, vacc=3.5,
           nsv=9, pdop=1.8, sacc=0.6)    # (an old second: still parsed)
gl.st["rx"] = _clock[0]
check("PVT-PARSE: latest epoch's speed, course, sats, fix type, accuracies",
      gl.st["used"] == 9 and gl.st["fixtype"] == 3 and gl.st["quality"] == 1
      and abs(gl.st["speed"] - 7.2) < 1e-9
      and abs(gl.st["course"] - 12.5) < 1e-9
      and abs(gl.st["gspeed"] - 2.0) < 1e-9
      and abs(gl.st["vacc"] - 3.5) < 1e-9 and abs(gl.st["sacc"] - 0.6) < 1e-9
      and abs(gl.st["pdop"] - 1.8) < 1e-9 and gl.status() == "3D fix",
      repr((gl.st["used"], gl.st["speed"], gl.st["course"])))


def pvt_case(**kw):
    gl.reset()
    pvt_second(T0, **kw)
    pvt_second(t_at(1), n=1, **kw)
    gl.st["rx"] = _clock[0]
    return (gl.st["valid"],
            bool(gl.has_fix(_clock[0])) and gl._usable(_clock[0]))


c_llh = pvt_case(flags3=1)
c_nofix = pvt_case(flags=0)
c_2d = pvt_case(fix=2)
ele_2d = gl._fields(_clock[0])[0]
c_dr = pvt_case(fix=5)                    # time only
c_far = pvt_case(hacc=25.0)
c_edge = pvt_case(hacc=20.0)
c_none = pvt_case(hacc_mm=0xFFFFFFFF)     # "unknown": ~2**32 mm
check("PVT-FILTER: invalidLlh, no gnssFixOK, time-only -> no fix",
      c_llh == (False, False) and c_nofix == (False, False)
      and c_dr == (False, False), repr((c_llh, c_nofix, c_dr)))
check("PVT-FILTER: 2D is a fix as GGA's is (no elevation in the line)",
      c_2d == (True, True) and ele_2d == "", repr((c_2d, ele_2d)))
check("PVT-FILTER: hAcc over PVT_MAX_HACC (or unknown) is a fix, not usable",
      c_far == (True, False) and c_edge == (True, True)
      and c_none == (True, False) and gl.PVT_MAX_HACC == 20,
      repr((c_far, c_edge, c_none)))

# PVT-FRAME: a cut NAV-PVT (another protocol's 84-byte payload) and a broken
# checksum are counted bad, not read
gl.reset()
_uart_rx[:] = []
good = pvt_frame(T0, 47.5, 19.0)
short = gl._ubx(0x01, 0x07, bytes(good[6:6 + 84]))
broken = bytearray(good)
broken[-1] ^= 0xFF
_uart_rx.extend(short + bytes(broken) + good)
gl.read_uart()
check("PVT-FRAME: truncated and bad-checksum frames counted, the good read",
      gl.st["ubx_bad"] == 2 and gl.st["ubx_ok"] == 1 and gl.ep["pn"] == 1,
      "bad=%d ok=%d pn=%d" % (gl.st["ubx_bad"], gl.st["ubx_ok"],
                              gl.ep["pn"]))
_uart_rx[:] = []

# PVT-AVG: integer offsets from the second's first fix weighted by 1/hAcc^2
gl.reset()
lats = [475000000, 475000010, 475000030, 475000005, 474999990]
lons = [190000000, 190000020, 189999990, 190000000, 190000013]
haccs = [1.0, 2.0, 4.0, 1.0, 0.0]          # (0: counted as 1 mm)
pvt_second(T0, lat=lats, lon=lons, hacc=haccs)
pvt_second(t_at(1), n=1)
ws = [1 / max(h, 0.001) ** 2 for h in haccs]


def wmean(v):
    return v[0] + gl._round(sum(w * (x - v[0]) for w, x in zip(ws, v))
                            / sum(ws))


check("PVT-AVG: the mean is weighted by 1/hAcc^2 (hAcc 0 guarded)",
      gl.st["lat_i"] == wmean(lats) and gl.st["lon_i"] == wmean(lons)
      and gl.st["lat_i"] == 474999990 and gl.st["epochs"] == 5,
      repr((gl.st["lat_i"], gl.st["lon_i"], wmean(lats), wmean(lons))))
gl.reset()
pvt_second(T0, lat=[475000000, 475000500, 475000510, 475000520, 475000530],
           hacc=[30.0, 2.0, 2.0, 2.0, 2.0])
pvt_second(t_at(1), n=1)
over = (gl.st["lat_i"], gl.st["epochs"])
gl.reset()
pvt_second(T0, lat=[475000000, 475000500, 475000510, 475000520, 475000530],
           hacc=[2.0, 2.0, 30.0, 2.0, 2.0])
pvt_second(t_at(1), n=1)
over2 = (gl.st["lat_i"], gl.st["epochs"], round(gl.st["hacc"], 3))
check("PVT-AVG: a fix over PVT_MAX_HACC is left out of a second with good "
      "ones (before or after them)",
      over == (475000515, 4) and over2 == (475000388, 4, 2.0),
      repr((over, over2)))
gl.reset()
pvt_second(T0, lat=lats, mps=[1.0, 1.0, 5.0, 1.0, 1.0],
           hacc=[3.0, 1.0, 1.0, 1.0, 1.0])
pvt_second(t_at(1), n=1)
check("PVT-AVG: above AVG_MAX_SPEED the point is the second's first fix",
      gl.st["lat_i"] == lats[0] and abs(gl.st["hacc"] - 3.0) < 1e-9
      and abs(gl.st["mps"] - 1.0) < 1e-9,
      repr((gl.st["lat_i"], gl.st["hacc"], gl.st["mps"])))

# PVT-I7: integer coordinates printed without a float, as "%.6f" would
i7 = [gl._i7(v) for v in (474979123, 474979125, -5, -4, 5, 4, 0, -15, 15,
                          -1512152967, 1799999995, -1799999995, -10, 9)]
# PVT-ACC: Dawarich keeps horizontal_accuracy as an integer; the M10's
# 0.4-0.5 m (SBAS) was stored as 0: acc goes in whole metres, rounded up,
# at least 1 (hdop keeps acc / HDOP_M)


def acc_field(h):
    gl.st.update(lat_i=475000000, lon_i=190000000, hacc=h, mps=1.0,
                 alt=None)
    f = gl._fields(_clock[0])
    return f[2], f[1]


acc_got = [acc_field(h) for h in (0.0, 0.4, 0.999, 1.0, 1.001, 2.5, 20.0)]
gl.reset()
check("PVT-ACC: acc in whole metres, rounded up, at least 1",
      [a for a, _ in acc_got] == ["1", "1", "1", "1", "2", "3", "20"]
      and acc_got[1][1] == "0.1", repr(acc_got))

check("PVT-I7: 1e-7 ints -> 6 decimals, halves away from zero, no -0",
      i7 == ["47.497912", "47.497913", "-0.000001", "0.000000", "0.000001",
             "0.000000", "0.000000", "-0.000002", "0.000002", "-151.215297",
             "180.000000", "-180.000000", "-0.000001", "0.000001"], repr(i7))

# PVT-SECONDS: 5 Hz epochs, one point a second; midnight; seconds queued
# behind a blocking call; an epoch reported as sec 59 + 0.999999 s
gl.reset()
d1 = (2026, 10, 18, 23, 59, 59)
pvt_second(d1, lat=[475000000 + k for k in range(5)])
gl._ubx_frame(pvt_frame((2026, 10, 18, 23, 59, 59), 47.6, 19.0,
                        nano=999999000))      # = 00:00:00.000 of the 19th
mid1 = (gl.st["time"], gl.st["date"], gl.st["seq"], gl.st["epochs"])
pvt_second((2026, 10, 19, 0, 0, 0), n=4, lat=47.6)
pvt_second((2026, 10, 19, 0, 0, 1), n=1, lat=47.7)
mid2 = (gl.st["time"], gl.st["date"], gl.st["seq"], gl.st["epochs"],
        gl.st["lat_i"])
check("PVT-SECONDS: 5 epochs a second publish once; across midnight dated",
      mid1 == ("23:59:59", "2026-10-18", 1, 5)
      and mid2 == ("00:00:00", "2026-10-19", 2, 5, 476000000),
      repr((mid1, mid2)))
q_td = tempfile.mkdtemp(prefix="pvtq-")
saved_dir = gl.LOG_DIR
try:
    gl.LOG_DIR = q_td
    gl.reset()
    gl.log_reset()
    _uart_rx[:] = []
    for s_ in range(4):                   # read after a blocking POST
        for k in range(5):
            _uart_rx.extend(pvt_frame(t_at(s_), 475000000 + 100 * s_,
                                      190000000, nano=k * 200000000))
    gl.read_uart()
    queued = len(gl.st["q"])
    gl.log_tick()
    q_lines = list(gl.log["batch"])
finally:
    gl.LOG_DIR = saved_dir
    shutil.rmtree(q_td, ignore_errors=True)
check("PVT-SECONDS: seconds published in one read are each a point",
      queued == 3 and len(q_lines) == 3
      and q_lines[0] == "47.500000,19.000000,100.0,2,0.0,20261031T120000Z\n"
      and q_lines[2].startswith("47.500020,19.000000,"), repr(q_lines))

# TRK2: a file opened on NAV-PVT fixes is "#TRK2", its lines written from
# the integers (no single-precision grid); readers take TRK1 and TRK2
t2_td = tempfile.mkdtemp(prefix="trk2-")
saved_dir = gl.LOG_DIR
try:
    gl.LOG_DIR = t2_td
    gl.reset()
    gl.log_reset()
    for s_ in range(4):                   # (the 4th publishes the 3rd)
        pvt_second(t_at(s_), lat=474979123 + 200 * s_, lon=190402456,
                   hacc=1.25, mps=1.44, hmsl=123.45)
        gl.log_tick()
    gl.flush()
    path2 = gl.log["file"]
    with open(path2) as f:
        raw2 = f.read()
    # a receiver restart: NMEA seconds in the TRK2 file keep its format
    gl.handle(rv_nmea("GPGGA,120004.00,4728.0000,N,01900.0000,E,1,08,1.2,"
                      "99.0,M,40.0,M,,"))
    gl.handle(rv_nmea("GPRMC,120004.00,A,4728.0000,N,01900.0000,E,0.0,,"
                      "311026,,,A"))
    gl.handle(rv_nmea("GPGGA,120005.00,4728.0000,N,01900.0000,E,1,08,1.2,"
                      "99.0,M,40.0,M,,"))
    gl.log_tick()
    nmea_in_2 = gl.log["batch"][-1]
    gl.flush()
    gl.log["file"] = None                 # (as _close_file leaves it)
    gl.log["created"] = False
    # and the other way: a TRK1 file (opened on NMEA) gets NAV-PVT seconds
    gl.reset()
    gl.log_reset()
    gl.handle(rv_nmea("GPGGA,130000.00,4728.0000,N,01900.0000,E,1,08,1.2,"
                      "99.0,M,40.0,M,,"))
    gl.handle(rv_nmea("GPRMC,130000.00,A,4728.0000,N,01900.0000,E,0.0,,"
                      "311026,,,A"))
    gl.handle(rv_nmea("GPRMC,130001.00,A,4728.0000,N,01900.0000,E,0.0,,"
                      "311026,,,A"))
    gl.log_tick()
    pvt_second((2026, 10, 31, 13, 0, 2), lat=474667777, lon=190000000,
               hacc=4.0)
    pvt_second((2026, 10, 31, 13, 0, 3), n=1)
    gl.log_tick()
    gl.flush()
    path1 = gl.log["file"]
    with open(path1) as f:
        raw1 = f.read()
    n2, ok2, size2, _o, eof2 = gl.read_batch(path2, 0)
    locs2 = json.loads(bytes(gl._body[:size2]))["locations"]
    body2 = bytes(gl._body[:size2])
    n1, ok1, size1, _o, eof1 = gl.read_batch(path1, 0)
    locs1 = json.loads(bytes(gl._body[:size1]))["locations"]
    cnt = gl.count_points()
    gl._append_bad(path2, 0, os.path.getsize(path2))
    with open(path2 + ".bad") as f:
        bad2 = f.read()
    nb, okb, sizeb, _o, _e = gl.read_batch(path2 + ".bad", 0)
    locsb = json.loads(bytes(gl._body[:sizeb]))["locations"]
finally:
    gl.LOG_DIR = saved_dir
    gl.log_reset()
    shutil.rmtree(t2_td, ignore_errors=True)
want2 = ("#TRK2\n"
         "47.497912,19.040246,123.5,2,1.4,20261031T120000Z\n"   # acc 1.2
         "47.497932,19.040246,123.5,2,1.4,20261031T120001Z\n"   # up: 2
         "47.497952,19.040246,123.5,2,1.4,20261031T120002Z\n#END\n")
check("TRK2: a NAV-PVT file is #TRK2 lat,lon,ele,acc,speed,time from ints",
      raw2 == want2, repr(raw2))
check("TRK2: NMEA seconds in a TRK2 file: acc = HDOP * HDOP_M, no speed",
      nmea_in_2 == "47.466667,19.000000,99.0,6.0,,20261031T120004Z\n",
      repr(nmea_in_2))
check("TRK2: a TRK1 file keeps TRK1 for NAV-PVT seconds (hdop = acc / 5)",
      raw1.startswith("#TRK1\n47.466667,19.000000,99.0,1.2,20261031T130000Z\n")
      and "47.466778,19.000000,100.0,0.8,20261031T130002Z\n" in raw1,
      repr(raw1))
check("TRK2: upload JSON - accuracy from acc, speed in m/s, altitude",
      (n2, ok2, eof2) == (5, 5, True)
      and locs2[0]["geometry"]["coordinates"] == [19.040246, 47.497912]
      and locs2[0]["properties"] == {
          "timestamp": "2026-10-31T12:00:00Z", "altitude": 123.5,
          "horizontal_accuracy": 2.0, "speed": 1.4,
          "device_id": gl.DEVICE_ID}
      and locs2[4]["properties"]["horizontal_accuracy"] == 6.0
      and "speed" not in locs2[4]["properties"]
      and b'"horizontal_accuracy":2.0,"speed":1.4,"device_id"' in body2,
      repr(body2[:300]))
check("TRK2: TRK1 lines still read as before (hdop * HDOP_M, no speed)",
      (n1, ok1, eof1) == (2, 2, True)
      and locs1[0]["properties"]["horizontal_accuracy"] == 6.0
      and locs1[1]["properties"]["horizontal_accuracy"] == 4.0
      and "speed" not in locs1[1]["properties"], json.dumps(locs1)[:300])
check("TRK2: count_points and the .bad copy take TRK2 lines",
      cnt == 7 and bad2.count("\n") == 5 and not bad2.startswith("#")
      and (nb, okb) == (5, 5) and locsb[0]["properties"]["speed"] == 1.4,
      "cnt=%d bad=%r" % (cnt, bad2[:80]))
# TRK2-READ: odd fields of a TRK2 line are dropped or the point skipped as
# they are on TRK1 lines; a 6th comma is no point line
rb2 = tempfile.mkdtemp(prefix="trk2r-")
try:
    p_ = os.path.join(rb2, "20261031_120000.trk")
    with open(p_, "wb") as f:
        f.write(b"#TRK2\n"
                b"47.5,19.0,,1.25,inf,20261031T120000Z\n"
                b"47.5,19.0,,,,20261031T120001Z\n"
                b"47.5,19.0,,\xff,,20261031T120002Z\n"
                b"47.5,19.0,,1.0,2.0,,20261031T120003Z\n"
                b"47.5,19.0,,x,02.50,20261031T120004Z\n#END\n")
    nr, okr, sizer, _o, _e = gl.read_batch(p_, 0)
    locr = json.loads(bytes(gl._body[:sizer]))["locations"]
finally:
    shutil.rmtree(rb2, ignore_errors=True)
check("TRK2-READ: canonical numbers, empty/inf/text fields left out, a bad "
      "byte or a 6th comma skips the line",
      (nr, okr) == (4, 3)
      and locr[0]["properties"].get("horizontal_accuracy") == 1.2
      and "speed" not in locr[0]["properties"]
      and "horizontal_accuracy" not in locr[1]["properties"]
      and locr[2]["properties"].get("speed") == 2.5
      and "horizontal_accuracy" not in locr[2]["properties"],
      repr((nr, okr, locr)))
t2g = trk2gpx.convert(["#TRK2\n",
                       "47.497912,19.040246,123.5,1.2,1.4,20261031T120000Z\n",
                       "47.5,19.0,,,,20261031T120001Z\n",
                       "47.5,19.0,10.0,0.9,20261031T120002Z\n"], "x")
check("TRK2: trk2gpx reads TRK2 lines (acc and speed as extensions)",
      'xmlns:esp="' in t2g
      and '<trkpt lat="47.497912" lon="19.040246"><ele>123.5</ele><time>'
          '2026-10-31T12:00:00Z</time><extensions><esp:acc>1.2</esp:acc>'
          '<esp:speed>1.4</esp:speed></extensions></trkpt>' in t2g
      and '<time>2026-10-31T12:00:01Z</time></trkpt>' in t2g
      and '<hdop>0.9</hdop></trkpt>' in t2g, t2g)
check("TRK2: trk2gpx output of TRK1 lines unchanged (no extension namespace)",
      "xmlns:esp" not in trk2gpx.convert(
          ["47.5,19.0,10.0,0.9,20261031T120002Z\n"], "x"))

# TRK2-RING: the way out rebuilt from the departure ring is the line
# _sample_line writes, NAV-PVT seconds (from the integers) and NMEA ones
# mixed, in a TRK2 and in a TRK1 file (single precision: mpy_unix_logger)
def ring_mixed(fmt):
    gl.reset()
    gl.log_reset()
    gl.log["fmt"] = fmt
    gl.log["batch"] = ["x"]               # (a file's lines: its format)
    gl.log["still"] = sl = gl._still_state()
    sl.update(active=True, anchor=(47.5, 19.0), aref=(47.5, 19.0), n=10)
    want = []
    for k in range(gl.STILL_DEP_MAX + 7):
        tm = "10:%02d:%02d" % (k // 60, k % 60)
        if k % 3:
            gl.st.update(valid=True, date="2026-10-31", time=tm, fixtype=3,
                         lat_i=474979123 + 271 * k, lon_i=-3141593 - 314 * k,
                         hacc=0.5 + k % 7 * 1.25, mps=k % 5 * 0.45 or None,
                         alt=123.45 + k, hdop=None, std=None)
            gl.st["lat"] = gl.st["lat_i"] / 1e7
            gl.st["lon"] = gl.st["lon_i"] / 1e7
        else:
            gl.st.update(valid=True, date="2026-10-31", time=tm, fixtype=3,
                         lat_i=None, lon_i=None, hacc=None, mps=None,
                         lat=47.49791234567 + k * 0.0000271828,
                         lon=-0.31415 - k * 0.0000314159, alt=-0.04,
                         hdop=1.25 + k % 3, hdop_at=_clock[0], std=None)
        want.append(gl._sample_line(_clock[0], None, None)[1])
        gl._ring_add(sl, _clock[0], (gl.st["lat"], gl.st["lon"]), 10.0 + k)
    gl.log["batch"] = []
    gl.log["file"] = "/nonexistent/x.trk"
    gl._still_leave()
    got = list(gl.log["batch"])
    gl.log_reset()
    return want[-gl.STILL_DEP_MAX:], got


w2, g2 = ring_mixed(2)
w1, g1 = ring_mixed(1)
check("TRK2-RING: the way out is the line _sample_line writes (TRK2 and "
      "TRK1 files, NAV-PVT and NMEA seconds)",
      w2 == g2 and w1 == g1 and g2[1].count(",") == 5
      and g1[1].count(",") == 4 and g2[1].startswith("47.498129,-0.314411,")
      and g1[1].startswith("47.498129,-0.314411,"),
      repr([(a, b) for a, b in zip(w2 + w1, g2 + g1) if a != b][:2]))
gl.reset()

# HIST: the day history the status page draws - each line flush wrote,
# appended to HIST_DIR/<UTC day>.trk (the track file is deleted once
# uploaded), the newest HIST_DAYS days kept, served by /hist?d=&o=
import tempfile as _tf
_hist_saved = (gl.HIST_DIR, gl.LOG_DIR, gl.HIST_REPLY)
_hd = _tf.mkdtemp(prefix="hist-")
gl.HIST_DIR = os.path.join(_hd, "hist")
gl.LOG_DIR = _hd


def _hl(day, hms, lat=47.185):
    return "%.6f,18.427300,100.0,2,1.4,%sT%sZ\n" % (lat, day, hms)


def _hread(day):
    try:
        with open(os.path.join(gl.HIST_DIR, day + ".trk")) as f:
            return f.read()
    except OSError:
        return None


try:
    lines = [_hl("20261030", "235958"), _hl("20261030", "235959"),
             _hl("20261031", "000000"), _hl("20261031", "000001")]
    gl._hist_add(lines + ["not written\n"], 4)
    gl._hist_add([_hl("20261031", "000002")], 1)
    h30, h31 = _hread("20261030"), _hread("20261031")
    check("HIST: lines go to their own UTC day's file, appended, only "
          "lines[:k]",
          h30 == "".join(lines[:2])
          and h31 == "".join(lines[2:]) + _hl("20261031", "000002"),
          repr((h30, h31)))
    for d_ in range(1, 10):               # 9 old days, then a new one
        with open(os.path.join(gl.HIST_DIR, "202610%02d.trk" % d_), "w") as f:
            f.write(_hl("202610%02d" % d_, "120000"))
    gl._hist_add([_hl("20261101", "080000")], 1)
    kept = sorted(os.listdir(gl.HIST_DIR))
    check("HIST: a new day's file drops the days beyond HIST_DAYS",
          len(kept) == gl.HIST_DAYS and kept[-1] == "20261101.trk"
          and kept[0] == "20261006.trk", repr(kept))
    free_saved = gl.free
    gl.free = lambda: gl.MIN_FREE
    try:
        gl._hist_add([_hl("20261101", "080001")], 1)
    finally:
        gl.free = free_saved
    check("HIST: nothing written while the flash is nearly full",
          _hread("20261101") == _hl("20261101", "080000"))
    # flush: the lines that landed in the track file go to the history too
    gl.log_reset()
    gl.log["file"] = os.path.join(_hd, "t.trk")
    gl.log["fmt"] = 2
    fl = [_hl("20261101", "090000"), _hl("20261101", "090001", 47.186)]
    gl.log["batch"] = list(fl)
    gl.flush()
    check("HIST: flush appends the lines it wrote to the day history",
          _hread("20261101") == _hl("20261101", "080000") + "".join(fl)
          and gl.log["batch"] == [] and gl.log["points"] == 2,
          repr(_hread("20261101")))
    gl.log_reset()

    class _HConn:
        def __init__(self):
            self.sent = b""

        def settimeout(self, t):
            pass

        def sendall(self, b):
            self.sent += bytes(b)

    def _hget(q):
        c = _HConn()
        gl._web_reply(c, b"GET /hist?" + q + b" HTTP/1.1\r\n\r\n")
        head, _, body = c.sent.partition(b"\r\n\r\n")
        return head.split(b"\r\n")[0], head, body

    full = _hread("20261101").encode()
    a = _hget(b"d=20261101&o=0")
    b = _hget(b"d=20261101&o=%d" % len(_hl("20261101", "080000")))
    gl.HIST_REPLY = 50
    cap = _hget(b"d=20261101&o=10")
    gl.HIST_REPLY = _hist_saved[2]
    end = _hget(b"d=20261101&o=99999")
    bad = [_hget(q)[0] for q in (b"d=2026110&o=0", b"d=../../x&o=0",
                                 b"d=20261101&o=-1", b"d=20261101&o=x",
                                 b"d=20250101&o=0")]
    check("HIST-WEB: /hist?d=&o= sends the day from byte o (Content-"
          "Length), HIST_REPLY at most, nothing past the end",
          a[0] == b"HTTP/1.0 200 OK" and a[2] == full
          and b"Content-Length: %d" % len(full) in a[1]
          and b[2] == full[len(_hl("20261101", "080000")):]
          and cap[2] == full[10:60] and end[0] == b"HTTP/1.0 200 OK"
          and end[2] == b"", repr((a[0], len(a[2]), len(cap[2]), end[2])))
    check("HIST-WEB: a bad day, offset or a missing day is a 404",
          bad == [b"HTTP/1.0 404 Not Found"] * 5, repr(bad))
finally:
    gl.HIST_DIR, gl.LOG_DIR, gl.HIST_REPLY = _hist_saved
    shutil.rmtree(_hd, ignore_errors=True)
    gl.reset()

# PVT-SETUP: the receiver's kind (MON-VER) picks the setup: M10 one
# CFG-VALSET (RAM) with NAV-PVT on and every NMEA sentence off, M8 legacy
# CFG-MSG, u-blox 6/7 NMEA as before


def pvt_setup(kind_frame, gps_pvt=True):
    saved = gl.GPS_PVT
    gl.GPS_PVT = gps_pvt
    try:
        gl.reset()
        _uart_rx[:] = []
        if kind_frame is not None:
            gl._ubx_frame(kind_frame)
        _uart_tx[:] = []
        gl.gps_setup()
        return [f for f in _uart_tx if f[2:3] == b"\x06"]
    finally:
        gl.GPS_PVT = saved


def nmea_set(frames):
    return [f for f in frames if f[2:4] == b"\x06\x01" and f[6] == 0xF0]


M10_VER = pvt_monver(b"000A0000", b"PROTVER=34.10")
M8_VER = pvt_monver(b"00080000", b"PROTVER=18.00")
U6_VER = pvt_monver(b"00040007", b"PROTVER 7.03")
F9_VER = pvt_monver(b"00190000", b"PROTVER=27.11")
fr10 = pvt_setup(M10_VER)
# CFG-VALSET payload by hand: version 0, layers 0x01 (RAM), 2 reserved,
# then key (U4 LE) + U1 value per item; keys from the u-blox M10 SPG 5.10
# interface description / SparkFun u-blox_config_keys.h
vs = bytes((0, 1, 0, 0))
for key, val in ((0x20910007, 1), (0x209100BB, 0), (0x209100AC, 0),
                 (0x209100C0, 0), (0x209100D4, 0), (0x209100C5, 0),
                 (0x209100CA, 0), (0x209100B1, 0)):
    vs += _struct.pack("<IB", key, val)
vs_frame = gl._ubx(0x06, 0x8A, vs)
# GLONASS on: CFG-SIGNAL-GLO_ENA 0x10310025, CFG-SIGNAL-GLO_L1_ENA
# 0x10310018 (M10 SPG 5.10 interface description / SparkFun
# u-blox_config_keys.h), RAM


def glo_frame(on):
    return gl._ubx(0x06, 0x8A, bytes((0, 1, 0, 0))
                   + _struct.pack("<IB", 0x10310025, on)
                   + _struct.pack("<IB", 0x10310018, on))


check("PVT-SETUP: M10 - one CFG-VALSET (RAM): NAV-PVT on, GGA/RMC/GSA/GST/"
      "GSV/GLL/VTG off, between the 1 Hz and 5 Hz rate frames",
      gl.st["mode"] == "pvt" and gl.st["setup_ok"] is True
      and vs_frame in fr10 and not nmea_set(fr10)
      and fr10[0] == gl._rate(1000) and fr10[-1] == gl._rate(200)
      and fr10[2] == glo_frame(1)
      and fr10.index(vs_frame) == 3 and len(fr10) == 5,
      repr([f.hex() for f in fr10]))
fr8 = pvt_setup(M8_VER)
want8 = ([gl._ubx(6, 1, bytes((0x01, 0x07, 1)))]
         + [gl._ubx(6, 1, bytes((0xF0, m, 0)))
            for m in (0x00, 0x04, 0x02, 0x07, 0x03, 0x01, 0x05)])
check("PVT-SETUP: M8 - legacy CFG-MSG: NAV-PVT every epoch, NMEA off",
      gl.st["mode"] == "pvt" and fr8[2:-1] == want8
      and not any(f[2:4] == b"\x06\x8a" for f in fr8),
      repr([f.hex() for f in fr8]))
fr6 = pvt_setup(U6_VER)
fr_unknown = pvt_setup(None)
want_nmea = [gl._ubx(6, 1, bytes((0xF0, m, r))) for m, r in gl.GPS_MSGS]
# CFG-MSG class 0x01 id 0x07 (NAV-PVT) rate 0, by hand: the NMEA setup's
# last message frame
pvt_stop = b"\xb5\x62\x06\x01\x03\x00\x01\x07\x00\x12\x50"
check("PVT-SETUP: u-blox 6 (and an unknown receiver) keep the NMEA setup",
      gl.st["mode"] == "nmea" and fr6[2:-1] == want_nmea + [pvt_stop]
      and fr_unknown[2:-1] == want_nmea + [pvt_stop],
      repr([f.hex() for f in fr6]))
fr_off = pvt_setup(M10_VER, gps_pvt=False)
check("PVT-SETUP: GPS_PVT=0 keeps NMEA on an M10",
      gl.st["mode"] == "nmea"
      and fr_off[2:-1] == [glo_frame(1)] + want_nmea + [pvt_stop])
fr9 = pvt_setup(F9_VER)
check("PVT-SETUP: kinds - protocol 23+ VALSET, 15-22 CFG-MSG, older NMEA",
      vs_frame in fr9
      and gl._gps_kind(pvt_monver(b"00000000", b"PROTVER=15.00")[6:-2])
      == "m8"
      and gl._gps_kind(pvt_monver(b"00070000", b"PROTVER 14.00")[6:-2])
      is None, repr([f.hex() for f in fr9]))
pv_env = []
saved = gl.GPS_PVT
for v_ in ("0 # nmea", "auto", "1", "x", None):
    gl.GPS_PVT = True if v_ != "auto" else False
    gl._gps_pvt_env({"GPS_PVT": v_} if v_ is not None else {})
    pv_env.append(gl.GPS_PVT)
gl.GPS_PVT = saved
import inspect as _inspect
run_src = _inspect.getsource(gl.run)
check("PVT-SETUP: .env GPS_PVT auto/1/0 (comment-safe), read before setup",
      pv_env == [False, True, True, True, True]
      and run_src.index("_gps_pvt_env") < run_src.index("gps_setup()"),
      repr(pv_env))

# GLO: GLONASS on an M10 (off by default there), one CFG-VALSET to RAM
# after NAV5, once per receiver start (a signal change restarts its GNSS
# part), off too with GPS_GLO=0, a NAK tolerated; no frame on others
diag_clear()
gl.DIAG = 60000
try:
    pvt_setup(M10_VER)
    g_on = (gl.st["glo_boots"], gl.st["setup_ok"])
    _uart_tx[:] = []
    gl.gps_setup()                        # a retry, same receiver start
    g_retry = glo_frame(1) in _uart_tx
    gl.st["gps_boots"] += 1               # restarted: back at its defaults
    _uart_tx[:] = []
    gl.gps_setup()
    g_again = glo_frame(1) in _uart_tx and gl.st["glo_boots"] == 1
    g_lines = [l for l in diag_lines() if l.startswith("gps glonass")]
finally:
    gl.DIAG = 0
check("GLO: an M10 gets GLONASS on once per receiver start, one log line",
      g_on == (0, True) and not g_retry and g_again
      and g_lines == ["gps glonass on"], repr((g_on, g_retry, g_lines)))
saved_glo = gl.GPS_GLO
gl.GPS_GLO = False
try:
    fr_glo0 = pvt_setup(M10_VER)
finally:
    gl.GPS_GLO = saved_glo
check("GLO: GPS_GLO=0 sends it off (RAM outlives an ESP reset)",
      glo_frame(0) in fr_glo0 and glo_frame(1) not in fr_glo0)
_uart_nak_frames.add(glo_frame(1))
try:
    fr_gnak = pvt_setup(M10_VER)
    gnak = (gl.st["glo_bad"], gl.st["setup_ok"], gl.st["mode"])
    _uart_tx[:] = []
    gl.st["gps_boots"] += 1
    gl.gps_setup()
    gnak_again = glo_frame(1) in _uart_tx
finally:
    _uart_nak_frames.discard(glo_frame(1))
check("GLO: a NAK - the rest of the setup goes on, not asked again this boot",
      gnak == (True, True, "pvt") and vs_frame in fr_gnak
      and fr_gnak[-1] == gl._rate(200) and not gnak_again, repr(gnak))
check("GLO: no GLONASS frame for an M8, a u-blox 6 or an unknown receiver",
      not any(f[2:4] == b"\x06\x8a" and f[6:8] == b"\x00\x01"
              and b"\x25\x00\x31\x10" in f
              for f in pvt_setup(M8_VER) + pvt_setup(U6_VER)
              + pvt_setup(None)))
glo_env = []
saved_glo = gl.GPS_GLO
for v_ in ("0", "1", "x", None):
    gl.GPS_GLO = v_ != "1"
    gl._gps_glo_env({"GPS_GLO": v_} if v_ is not None else {})
    glo_env.append(gl.GPS_GLO)
gl.GPS_GLO = saved_glo
check("GLO: .env GPS_GLO 1/0, anything else keeps the default, read before "
      "setup",
      glo_env == [False, True, True, True]
      and run_src.index("_gps_glo_env") < run_src.index("gps_setup()"),
      repr(glo_env))
# GPS_PVT=0 on an M10: MON-VER after the NMEA boot setup makes a setup due
# for GLONASS alone (the mode stays NMEA)
saved = gl.GPS_PVT
gl.GPS_PVT = False
gl.reset()
_uart_rx[:] = []
gl.gps_setup()
_uart_ver[0] = M10_VER
_uart_tx[:] = []
try:
    gl.gps_check(_clock[0])
    gl.read_uart()
    for _ in range(8):
        _clock[0] += 1000
        gl.gps_check(_clock[0])
finally:
    _uart_ver[0] = None
    gl.GPS_PVT = saved
check("GLO: GPS_PVT=0 - MON-VER of an M10 makes the GLONASS setup due",
      glo_frame(1) in _uart_tx and gl.st["mode"] == "nmea"
      and gl.st["glo_boots"] == gl.st["gps_boots"] and vs_frame not in _uart_tx,
      repr((gl.st["mode"], gl.st["glo_boots"])))

# PVT-BOOT: the boot setup is NMEA (model unknown); MON-VER naming an M10
# makes the PVT setup due (within GPS_RETRY), not a second NMEA one
gl.reset()
_uart_rx[:] = []
gl.gps_setup()
mode0 = gl.st["mode"]
_uart_ver[0] = M10_VER
_uart_tx[:] = []
try:
    gl.gps_check(_clock[0])               # polls MON-VER: the answer
    gl.read_uart()
    for _ in range(8):
        _clock[0] += 1000
        gl.gps_check(_clock[0])
finally:
    _uart_ver[0] = None
check("PVT-BOOT: MON-VER of an M10 after an NMEA setup -> the PVT setup",
      mode0 == "nmea" and gl.st["mode"] == "pvt" and gl.st["gps_kind"] == "m10"
      and vs_frame in _uart_tx and gl.st["ack_n"] == 0,
      "%r %r" % (mode0, gl.st["mode"]))

# PVT-CHECK: _setup_check counts NAV-PVT epochs (no fix needed, no GST);
# one a second is not taken; none at all by GPS_VERIFY: NMEA for this boot


def pvt_verify(n_per_s, secs=12, frames=True):
    pvt_setup(M10_VER)
    _uart_tx[:] = []
    for k in range(secs):
        if frames:
            pvt_second(t_at(k), n=n_per_s, flags=0, fix=0)   # no fix
        else:
            gl.handle(rv_nmea("GPRMC,1200%02d.00,V,,,,,,,311026,,,N" % k))
        _clock[0] += 1000
        gl.gps_check(_clock[0])
    return [f for f in _uart_tx if f[2:3] == b"\x06"]


diag_clear()
gl.DIAG = 60000
try:
    v5 = pvt_verify(5)
    v5_state = (gl.st["checked"], gl.st["extra"], gl.st["pvt_bad"])
    v1 = pvt_verify(1)
    v1_state = (gl.st["extra"] > 0 or bool(v1), gl.st["pvt_bad"])
    vn = pvt_verify(5, secs=25, frames=False)
    vn_state = (gl.st["pvt_bad"], gl.st["mode"])
    fb_lines = [l for l in diag_lines() if l.startswith("gps pvt")]
finally:
    gl.DIAG = 0
check("PVT-CHECK: 5 NAV-PVT epochs a second without a fix -> taken, no GST",
      v5 == [] and v5_state == (True, 0, False), repr((v5, v5_state)))
check("PVT-CHECK: one NAV-PVT a second -> not taken, set up again",
      v1_state == (True, False) and vs_frame in v1, repr(v1_state))
check("PVT-CHECK: no NAV-PVT by GPS_VERIFY -> NMEA setup, PVT off this boot",
      vn_state == (True, "nmea") and nmea_set(vn)[-len(want_nmea):]
      == want_nmea and fb_lines == ["gps pvt no NAV-PVT, NMEA this boot"],
      repr((vn_state, fb_lines)))
pvt_setup(M10_VER)
gl.st["pvt_bad"] = True                   # (given up earlier this boot)
_uart_tx[:] = []
gl.gps_setup()
check("PVT-CHECK: given up -> NMEA setups for the rest of the boot",
      gl.st["mode"] == "nmea" and vs_frame not in _uart_tx)

# NMEA-PVT-STOP: an ESP reset leaves the receiver powered, NAV-PVT on from
# the NAV-PVT boot before (a RAM VALSET). GPS_PVT=0: the NMEA setup turns
# it off (CFG-MSG, the model not known yet); a NAK of that frame alone (a
# u-blox 6 has no NAV-PVT) is no failed setup; a NAV-PVT frame while in
# the NMEA mode is not used and makes the setup due again
_uart_nak_frames.add(pvt_stop)
try:
    fr_u6nak = pvt_setup(U6_VER, gps_pvt=False)
    u6nak = (gl.st["setup_ok"], gl.st["mode"])
finally:
    _uart_nak_frames.discard(pvt_stop)
check("NMEA-PVT-STOP: a NAK of the NAV-PVT stop alone -> setup ok, 5 Hz",
      u6nak == (True, "nmea") and fr_u6nak[-2] == pvt_stop
      and fr_u6nak[-1] == gl._rate(1000 // gl.GPS_HZ),
      repr((u6nak, [f.hex() for f in fr_u6nak[-2:]])))
pvt_setup(None, gps_pvt=False)            # boot: model unknown
gl.st["checked"] = True
_clock[0] += 2000
ubx0 = gl.st["ubx_ok"]
pvt_second(t_at(0), lat=47.6, lon=19.1)    # still on in the receiver
left_on = (gl.st["valid"], gl.st["lat_i"], gl.st["fix_src"], gl.st["extra"],
           gl.st["ubx_ok"] - ubx0)
_uart_tx[:] = []
for _ in range(gl.GPS_RETRY // 1000 + 1):
    _clock[0] += 1000
    gl.gps_check(_clock[0])
check("NMEA-PVT-STOP: NAV-PVT in the NMEA mode -> not used, set up again",
      left_on[:3] == (False, None, None) and left_on[3] == 5
      and left_on[4] == 5 and pvt_stop in _uart_tx
      and gl.st["mode"] == "nmea", repr(left_on))
_uart_nak.add((0x06, 0x8A))
try:
    fr_nak = pvt_setup(M10_VER)
    nak_state = (gl.st["pvt_bad"], gl.st["setup_ok"])
    _uart_tx[:] = []
    _clock[0] += gl.GPS_RETRY + 1000
    gl.gps_check(_clock[0])
finally:
    _uart_nak.discard((0x06, 0x8A))
check("PVT-CHECK: a NAK of the CFG-VALSET -> PVT off, the NMEA setup next",
      nak_state == (True, False) and gl.st["mode"] == "nmea"
      and nmea_set(_uart_tx) == want_nmea, repr(nak_state))

# PVT-RESTART: a restarted receiver is back at NMEA: its banner counts, its
# NMEA positions keep the track going, and the setup turns PVT on again
pvt_setup(M10_VER)
gl.st["checked"] = True
_uart_tx[:] = []
_uart_rx[:] = []
_uart_rx.extend(rv_nmea("GPTXT,01,01,02,u-blox ag - www.u-blox.com")
                + b"\r\n")
for k in range(3):
    _uart_rx.extend(rv_nmea("GPGGA,1200%02d.00,4730.000,N,01900.000,E,1,08,"
                            "1.0,100.0,M,40.0,M,," % k) + b"\r\n")
    _uart_rx.extend(rv_nmea("GPRMC,1200%02d.00,A,4730.000,N,01900.000,E,"
                            "0.1,,311026,,,A" % k) + b"\r\n")
gl.read_uart()
restart = (gl.st["gps_boots"], gl.st["fix_src"], gl.st["valid"],
           gl.st["extra"] > 0)
for _ in range(gl.GPS_RETRY // 1000 + 1):
    _clock[0] += 1000
    gl.gps_check(_clock[0])
check("PVT-RESTART: banner counted, NMEA fixes used, PVT set up again",
      restart == (1, "nmea", True, True) and vs_frame in _uart_tx
      and gl.st["mode"] == "pvt" and gl.st["gps_kind"] == "m10",
      repr(restart))
_uart_rx[:] = []
gl.reset()


# WiFi: findings 1, 3, 8, 9, 22, 23 and the WiFi improvements
rv_td = tempfile.mkdtemp(prefix="rev29-")


def rv_nets(home_ip=None):
    return [{"ssid": "home", "pw": "x", "home": True, "hidden": False,
             "bssid": None, "ip": home_ip},
            {"ssid": "hot", "pw": "y", "home": False, "hidden": False,
             "bssid": None, "ip": None}]


def rv_connect(ok, aps, nets=None):
    online(rv_td)
    gl.wifi["nets"] = nets or rv_nets()
    WLAN.ok = set(ok)
    WLAN.scan_aps = aps
    gl.wifi["wlan"] = w = WLAN(0)
    w.active(True)
    gl._wifi_begin_round(w)
    for _ in range(30):
        _clock[0] += 500
        gl.wifi_tick()
        if gl.wifi["state"] == "connected":
            break
    return w


saved_rv = (gl.WEB, gl.WEB_AWAY, WLAN.scan_aps, set(WLAN.ok), gl.PB_MODE,
            gl.PB_CONNECTED)
try:
    # W-1: a Latin-1 / cut-off UTF-8 SSID in a scan does not crash
    latin = ("K\xe1v\xe9z\xf3", -60, b"\x04" * 6)
    w = rv_connect([], [])
    w.active(True)
    WLAN.scan_aps = []
    real_scan = WLAN.scan
    WLAN.scan = lambda self: [(b"K\xe1v\xe9z\xf3", b"\x04" * 6, 1, -60, 4,
                               False), (b"hot", b"\x05" * 6, 1, -50, 4, False)]
    try:
        crash = None
        gl.log["still"] = sl = gl._still_state()
        sl.update(active=True, anchor=(47.5, 19.0), aref=(47.5, 19.0), n=100)
        try:
            found = gl._scan_known(w)
            for _ in range(8):
                gl._stay_scan(w.scan())
        except Exception as e:
            crash = e
    finally:
        WLAN.scan = real_scan
    check("W-1: a non-UTF-8 SSID in a scan neither crashes nor matches",
          crash is None and found is not None
          and [n["ssid"] for n in found] == ["hot"]
          and b"\x04" * 6 in gl.log["still"]["aps"], repr(crash))

    # W-3: an away stay on a hotspot lets the radio sleep once uploaded
    gl.WEB, gl.WEB_AWAY = True, False
    w = rv_connect(["hot"], [("hot", -50), ("cafe", -60, b"\x04" * 6)])
    conn = gl.wifi["state"] == "connected" and not gl.wifi["home"]
    gl.log["clock"] = (800000000, _clock[0])
    gl.log["still"] = sl = gl._still_state()
    sl.update(active=True, anchor=(47.5, 19.0), aref=(47.5, 19.0), n=100)
    states = []
    for _ in range(20 * 60):
        _clock[0] += 1000
        gl.log_tick()
        gl.wifi_tick()
        gl.upload_tick()
        if not states or states[-1] != gl.wifi["state"]:
            states.append(gl.wifi["state"])
    check("W-3: an away stay on a hotspot: the radio sleeps once idle",
          conn and "sleep" in states and gl.log["file"] is not None,
          repr(states))

    # W-8: WEB at home: a keep-alive scan seeing home starts a round
    gl.WEB = True
    w = rv_connect(["home"], [("home", -50)])
    WLAN.ok, WLAN.scan_aps = set(), []
    gl._wifi_off(w, "sleep")             # as after a failed round
    WLAN.ok, WLAN.scan_aps = {"home"}, [("home", -50)]
    t0 = _clock[0]
    back = None
    for _ in range(5 * 60):
        _clock[0] += 1000
        gl.keepalive_tick(_clock[0])
        gl.wifi_tick()
        if gl.wifi["state"] == "connected":
            back = (_clock[0] - t0) // 1000
            break
    check("W-8: WEB, radio asleep, home seen by a keep-alive scan -> back",
          back is not None and back <= 60, "back after %r s" % back)

    # W-9: every waiting track held back -> the radio may sleep until then
    gl.WEB = False
    w = rv_connect(["hot"], [("hot", -50)])
    trk = os.path.join(rv_td, "20261029_100000.trk")
    make_trk(trk, 3)
    now = _clock[0]
    gl.up["hold"] = {trk: now + 1800000}
    gl.up["state"] = "held"
    wait = gl._upload_wait(now)
    gl.up["hold"] = {trk: now - 1}
    due = gl._upload_wait(now)
    gl.up["hold"] = {}
    os.remove(trk)
    check("W-9: all tracks held -> wait until the first hold ends",
          1790000 <= wait <= 1800000 and due == 0, "%r %r" % (wait, due))

    # W-22: lost -> offline stops the IDF reconnect before the round
    w = rv_connect(["hot"], [("hot", -50)])
    gl.wifi["state"] = "lost"
    gl.wifi["try_at"] = _clock[0] - 10 * gl.LOST_GRACE
    w._status = 5
    w._requested = True
    WLAN.ok = set()
    calls = []
    real_disc = getattr(WLAN, "disconnect", None)
    real_scan2 = WLAN.scan
    WLAN.disconnect = lambda self: (calls.append("disc"),
                                    real_disc(self) if real_disc else None)
    WLAN.scan = lambda self: (calls.append("scan"), real_scan2(self))[1]
    try:
        gl.wifi_tick()
    finally:
        if real_disc:
            WLAN.disconnect = real_disc
        WLAN.scan = real_scan2
    check("W-22: lost -> offline disconnects before the round's scan",
          "scan" in calls and "disc" in calls
          and calls.index("disc") < calls.index("scan"), repr(calls))

    # W-23: no .env: gpio pulses still go out; scan pulses get a WLAN
    gl.wifi_reset()
    gl.wifi["state"] = "no .env"
    gl.PB_MODE = "gpio"
    pins = []
    real_gpio = gl._pb_pulse_gpio
    gl._pb_pulse_gpio = lambda: pins.append(1) or True
    try:
        for _ in range(4):
            _clock[0] += gl.PB_KEEPALIVE
            gl.keepalive_tick(_clock[0])
    finally:
        gl._pb_pulse_gpio = real_gpio
    gl.PB_MODE = "scan"
    gl._pb_w = None
    n0 = gl.wifi["pb_scans"]
    _clock[0] += gl.PB_KEEPALIVE
    gl.keepalive_tick(_clock[0])
    check("W-23: without .env the keep-alive still pulses (gpio and scan)",
          len(pins) == 4 and gl.wifi["pb_scans"] == n0 + 1
          and gl._pb_w is not None, "%d %d" % (len(pins), gl.wifi["pb_scans"]))

    # KEEPALIVE_CONNECTED: pulses while connected and idle, opt-in
    w = rv_connect(["home"], [("home", -50)])
    gl.PB_CONNECTED = False
    n0 = gl.wifi["pb_scans"]
    for _ in range(3):
        _clock[0] += gl.PB_KEEPALIVE
        gl.keepalive_tick(_clock[0])
    off_n = gl.wifi["pb_scans"] - n0
    gl._keepalive_env({"KEEPALIVE_CONNECTED": "1"})
    for _ in range(3):
        _clock[0] += gl.PB_KEEPALIVE
        gl.keepalive_tick(_clock[0])
    on_n = gl.wifi["pb_scans"] - n0
    check("KEEPALIVE_CONNECTED: off by default; on, scans keep the link",
          off_n == 0 and on_n == 3 and gl.wifi["state"] == "connected"
          and w.active(), "%d %d %s" % (off_n, on_n, gl.wifi["state"]))

    # DHCP after a static network: always asked for once any net is static
    gl.PB_CONNECTED = False
    cfgs = []
    real_ifc = getattr(WLAN, "ifconfig", None)

    def rec_ifc(self, *a):
        if a:
            cfgs.append(a[0])
            return None
        return ("192.168.1.99", "255.255.255.0", "192.168.1.1", "1.1.1.1")
    WLAN.ifconfig = rec_ifc
    try:
        gl.wifi["static"] = False        # a soft reset forgot the flag
        gl.wifi["nets"] = rv_nets(("192.168.1.60", "255.255.255.0",
                                   "192.168.1.1", "192.168.1.1"))
        gl._wifi_ip(w, gl.wifi["nets"][1])
    finally:
        if real_ifc:
            WLAN.ifconfig = real_ifc
    check("IP: a DHCP network asks for DHCP when another one is static",
          cfgs == ["dhcp"], repr(cfgs))

    # hidden home: a failed try on a neighbour's blank AP is not repeated
    gl.wifi_reset()
    gl.wifi["nets"] = [{"ssid": "home", "pw": "x", "home": True,
                        "hidden": True, "bssid": None, "ip": None}]
    gl.wifi.update(blank_rssi=-40, blank_bssid=b"\x10" * 6, roaming=True)
    ok_before = gl._blank_home_ok()
    gl._mark_failed("home", "test")
    ok_after = gl._blank_home_ok()
    gl.wifi["blank_bssid"] = b"\x11" * 6
    ok_other = gl._blank_home_ok()
    check("HIDDEN: a blank AP whose home try failed is not tried again",
          ok_before and not ok_after and ok_other,
          "%r %r %r" % (ok_before, ok_after, ok_other))

    # WR-4: a failed roam to the hidden home blamed that blank AP until
    # reboot (it may be our own router, briefly unreachable): the blame
    # ends after BLANK_BAD_HOLD and on any hidden-home connect
    gl.wifi_reset()
    gl.wifi["nets"] = [{"ssid": "home", "pw": "x", "home": True,
                        "hidden": True, "bssid": None, "ip": None}]
    gl.wifi["wlan"] = wb = WLAN(0)
    gl.wifi.update(blank_rssi=-50, blank_bssid=b"\x12" * 6, roaming=True)
    gl._mark_failed("home", "test")
    held = not gl._blank_home_ok()
    _clock[0] += getattr(gl, "BLANK_BAD_HOLD", 3600000)
    expired = gl._blank_home_ok()
    gl.wifi["roaming"] = True
    gl._mark_failed("home", "test")
    again = not gl._blank_home_ok()
    gl.wifi["ssid"] = "home"
    gl._wifi_got_ip(wb)
    cleared = gl._blank_home_ok()
    check("WR-4: a blamed blank AP is cleared by time and by a home connect",
          held and expired and again and cleared,
          "%r %r %r %r" % (held, expired, again, cleared))

    # WR-5: connected to the hidden home, its BSSID is learned (v1.29 has
    # no config("bssid"): the one blank AP on the link's channel near its
    # RSSI); then a stranger's hidden AP is no longer "home in range" (it
    # vetoed departures), ours still is. Nothing learned: as before
    hid_nets = [{"ssid": "home", "pw": "x", "home": True, "hidden": True,
                 "bssid": None, "ip": None},
                {"ssid": "hot", "pw": "y", "home": False, "hidden": False,
                 "bssid": None, "ip": None}]
    mine_b, neigh_b = b"\x41" * 6, b"\x42" * 6
    w = rv_connect(["home"], [("", -55, mine_b), ("hot", -40)],
                   nets=[dict(n) for n in hid_nets])
    conn = gl.wifi["state"] == "connected" and gl.wifi["ssid"] == "home"
    gl.wifi_tick()                       # the learning scan
    learned = list(gl.wifi.get("home_aps") or [])
    gl._wifi_off(w, "sleep")
    WLAN.scan_aps = [("", -40, neigh_b), ("hot", -40)]
    stranger = gl.home_in_range()
    WLAN.scan_aps = [("", -60, mine_b), ("", -40, neigh_b)]
    ours = gl.home_in_range()
    check("WR-5: the hidden home's BSSID is learned; strangers' blank APs "
          "no longer count as home", conn and learned == [mine_b]
          and stranger is False and ours is True,
          "%r %r %r %r" % (conn, learned, stranger, ours))
    w = rv_connect(["home"], [("", -55, mine_b), ("", -52, neigh_b),
                              ("hot", -40)], nets=[dict(n) for n in hid_nets])
    gl.wifi_tick()
    amb = list(gl.wifi.get("home_aps") or [])
    gl._wifi_off(w, "sleep")
    WLAN.scan_aps = [("", -40, neigh_b)]
    before = gl.home_in_range()
    check("WR-5: two blank APs that fit: nothing learned, any counts as before",
          amb == [] and before is True, "%r %r" % (amb, before))

    # WR-29: no GEO_URL/API_KEY ("no .env") or a full .bad ("bad full"):
    # the radio never went off away from home
    w = rv_connect(["hot"], [("hot", -50)])
    trk = os.path.join(rv_td, "20261031_100000.trk")
    make_trk(trk, 3)
    gl.up["url"] = None
    for _ in range(gl.WIFI_IDLE // 1000 + 5):
        _clock[0] += 1000
        gl.upload_tick()
        gl.wifi_tick()
    noenv = (gl.up["state"], gl.wifi["state"])
    gl.up["url"] = "https://dawarich.example.com"
    now = _clock[0]
    gl.up.update(state="bad full", next_at=now + gl.BACKOFF_START,
                 hold={trk: now + gl.BACKOFF_MAX})
    wait = gl._upload_wait(now)
    gl.up["hold"] = {}
    os.remove(trk)
    check("WR-29: 'no .env' lets the radio sleep; 'bad full' waits for the hold",
          noenv == ("no .env", "sleep") and wait >= gl.BACKOFF_MAX - 1000,
          "%r %r" % (noenv, wait))

    # WR-30: WEB, radio asleep: a keep-alive scan seeing the hidden home's
    # learned BSSID starts a round (it only knew visible SSIDs)
    gl.WEB, gl.PB_MODE = True, "scan"
    w = rv_connect(["home"], [("", -55, mine_b), ("hot", -40)],
                   nets=[dict(n) for n in hid_nets])
    gl.wifi_tick()                       # learn
    WLAN.ok, WLAN.scan_aps = set(), []
    gl._wifi_off(w, "sleep")
    WLAN.ok, WLAN.scan_aps = {"home"}, [("", -55, mine_b)]
    t0, back = _clock[0], None
    for _ in range(5 * 60):
        _clock[0] += 1000
        gl.keepalive_tick(_clock[0])
        gl.wifi_tick()
        if gl.wifi["state"] == "connected":
            back = (_clock[0] - t0) // 1000
            break
    check("WR-30: a keep-alive scan seeing the hidden home's BSSID wakes it",
          back is not None and back <= 60, "back after %r s" % back)

    # WR-30: WEB on, gpio keep-alive (no scan sees home come back): a failed
    # round searches on with the backoff for WEB_SEARCH, then sleeps; it
    # dropped to WIFI_SLEEP (status page down 1 h) at once
    gl.PB_MODE = "gpio"
    w = rv_connect([], [])
    for _ in range(60):
        if gl.wifi["state"] != "connecting":
            break
        _clock[0] += 1000
        gl.wifi_tick()
    first = gl.wifi["state"]
    WLAN.ok, WLAN.scan_aps = {"home"}, [("home", -50)]
    t0, back = _clock[0], None
    for _ in range(10 * 60):
        _clock[0] += 1000
        gl.wifi_tick()
        if gl.wifi["state"] == "connected":
            back = (_clock[0] - t0) // 1000
            break
    w = rv_connect([], [])
    for _ in range(45 * 60 // 5):
        _clock[0] += 5000
        gl.wifi_tick()
    check("WR-30: WEB without scan keep-alive: a failed round searches on, "
          "then sleeps", first == "not found" and back is not None
          and back <= 120 and gl.wifi["state"] == "sleep",
          "%r %r %r" % (first, back, gl.wifi["state"]))
    gl.WEB, gl.PB_MODE = False, "scan"

    # WR-e: wifi_tick asked any(home) of every configured network on every
    # loop pass: cached per nets list
    class WrNets(list):
        n = 0

        def __iter__(self):
            WrNets.n += 1
            return list.__iter__(self)
    w = rv_connect(["hot"], [("hot", -50)])
    gl.wifi["nets"] = WrNets(gl.wifi["nets"])
    for _ in range(50):
        _clock[0] += 100
        gl.wifi_tick()
    check("WR-e: the 'any home network' flag is not rebuilt every pass",
          WrNets.n <= 2 and gl.wifi["state"] == "connected", "%d" % WrNets.n)

    # WR-3: the esp32 port raises RuntimeError ("Wifi Unknown Error 0x...")
    # for an unmapped IDF error (no memory in a scan); every WLAN call site
    # caught OSError only -> machine.reset(), five early ones -> REPL
    def rt_err(*a, **k):
        raise RuntimeError("Wifi Unknown Error 0x101")
    names = ("scan", "active", "connect", "disconnect", "status", "ifconfig",
             "config")
    real_w = dict((k, WLAN.__dict__[k]) for k in names)
    real_ctor = sys.modules["network"].WLAN
    gl.PB_MODE, gl.PB_CONNECTED = "scan", True
    w = rv_connect(["hot"], [("hot", -50)])
    crashed = []

    def rt_try(label, fn):
        try:
            fn()
        except Exception as e:
            crashed.append("%s %r" % (label, e))
    for k in names:
        setattr(WLAN, k, rt_err)
    try:
        _clock[0] += gl.PB_KEEPALIVE
        rt_try("keepalive connected", lambda: gl.keepalive_tick(_clock[0]))
        rt_try("wifi_tick connected", gl.wifi_tick)
        gl.wifi["state"] = "sleep"
        rt_try("home_in_range", gl.home_in_range)
        _clock[0] += gl.PB_KEEPALIVE
        rt_try("keepalive asleep", lambda: gl.keepalive_tick(_clock[0]))
        gl.wifi["state"] = "not found"
        gl.wifi["last_check"] = _clock[0] - gl.wifi["backoff"]
        rt_try("wifi_tick round", gl.wifi_tick)
        gl.wifi["state"] = "connecting"
        rt_try("wifi_tick connecting", gl.wifi_tick)
        rt_try("wifi_stop", gl.wifi_stop)
        sys.modules["network"].WLAN = rt_err
        gl._pb_w = None
        rt_try("pb_wlan", gl._pb_wlan)
        write_env(rv_td, "SSID=home\nWPA2=x\n")
        gl.wifi_reset()
        rt_try("wifi_start", gl.wifi_start)
    finally:
        for k in names:
            setattr(WLAN, k, real_w[k])
        sys.modules["network"].WLAN = real_ctor
        gl._pb_w = None
    check("WR-3: a RuntimeError from any WLAN call does not escape",
          crashed == [] and gl.wifi["state"] == "error", repr(crashed))

    # R2: WiFi review 2 (hidden home, unlisted networks, "no .env")
    R2_OURS, R2_NEIGH, R2_MESH = b"\x51" * 6, b"\x52" * 6, b"\x53" * 6
    r2_saved = (WLAN.assoc_channel, gl.PB_KEEPALIVE, gl.HOME_APS_FILE,
                gl.free)
    r2_haf = os.path.join(rv_td, "home_aps")
    r2_env = ("SSID=home\nWPA2=pw\nSSID_HIDDEN=1\nSSID2=phone\nWPA2_2=pw2\n"
              "GEO_URL=https://d.example\nAPI_KEY=k\n")

    def r2_boot(env_text, aps, ok):
        reset_wlan()
        WLAN.scan_aps = aps
        WLAN.ok = set(ok)
        write_env(rv_td, env_text)
        gl.LOG_DIR = rv_td
        gl.log_reset()
        gl.wifi_reset()
        gl.up_reset()
        gl.wifi_start()
        return gl.wifi["wlan"]

    def r2_until(cond, secs, step=1000, pulse=False):
        for _ in range(secs * 1000 // step):
            _clock[0] += step
            if pulse:
                gl.keepalive_tick(_clock[0])
            gl.wifi_tick()
            if cond():
                return True
        return False

    try:
        gl.WEB = False
        # R2-1: the link drops between GOT_IP and the next wifi_tick (the
        # first POST), then a hotspot connects: its tick learned a
        # neighbour's blank AP on the hotspot's channel as the hidden home
        w = r2_boot(r2_env, [("", -55, R2_OURS, 6)], ["home"])
        first = (gl.wifi["state"], gl.wifi["ssid"])
        w._status = 0
        w._requested = False
        WLAN.ok = {"phone"}
        WLAN.scan_aps = [("phone", -60, b"\x06" * 6, 11),
                         ("", -62, R2_NEIGH, 11)]
        WLAN.assoc_channel = 11
        _clock[0] += 1000
        gl.wifi_tick()                   # connected -> lost
        lost_learn = gl.wifi.get("learn")
        r2_until(lambda: gl.wifi["state"] == "connected", 120)
        gl.wifi_tick()
        gl.wifi_tick()
        check("R2-1: a drop before the learning tick and a hotspot connect "
              "learn no neighbour's blank AP", first == ("connected", "home")
              and not lost_learn and gl.wifi["ssid"] == "phone"
              and gl.wifi["home_aps"] == [] and not gl.wifi.get("learn"),
              "%r %r %s %r" % (first, lost_learn, gl.wifi["ssid"],
                               gl.wifi["home_aps"]))
        WLAN.assoc_channel = 1

        # R2-1: a wrong BSSID in the list (learned once, or from the file)
        # goes after HOME_AP_MISS hidden-home connects through another one;
        # one the scan shows with a visible SSID goes at once. It stayed in
        # the MRU list (veto, wake) for good
        hid = [{"ssid": "home", "pw": "x", "home": True, "hidden": True,
                "bssid": None, "ip": None},
               {"ssid": "hot", "pw": "y", "home": False, "hidden": False,
                "bssid": None, "ip": None}]
        aps, miss = [R2_NEIGH], {}
        n_miss = getattr(gl, "HOME_AP_MISS", 5)
        hist = []
        for _ in range(n_miss):
            rv_connect(["home"], [("", -50, R2_OURS)],
                       nets=[dict(n) for n in hid])
            gl.wifi["home_aps"], gl.wifi["home_miss"] = aps, miss
            gl.wifi_tick()               # the learning scan
            aps = list(gl.wifi["home_aps"])
            miss = gl.wifi.get("home_miss") or {}
            hist.append(R2_NEIGH in aps)
        rv_connect(["home"], [("", -50, R2_OURS), ("cafe", -70, R2_MESH)],
                   nets=[dict(n) for n in hid])
        gl.wifi["home_aps"] = [R2_OURS, R2_MESH]
        gl.wifi_tick()
        vis = list(gl.wifi["home_aps"])
        check("R2-1: a wrongly learned BSSID does not stick",
              hist == [True] * (n_miss - 1) + [False]
              and aps == [R2_OURS] and vis == [R2_OURS],
              "%r %r %r" % (hist, aps, vis))

        # R2-7: the hidden home's other node (mesh) is not learned yet:
        # still enough to roam home from a hotspot (it was filtered out once
        # one BSSID was learned), not for "home in range"; connected through
        # it, it is learned too. An AP the IDF names with the hidden SSID
        # while connected is taken over a blank one that only fits
        w = rv_connect(["hot"], [("hot", -40)], nets=[dict(n) for n in hid])
        on_hot = gl.wifi["ssid"] == "hot"
        gl.wifi["home_aps"] = [R2_OURS]
        WLAN.scan_aps = [("", -52, R2_MESH), ("hot", -40)]
        WLAN.ok = {"home", "hot"}
        WLAN.connects = []
        gl.st["speed"] = None
        gl.wifi["roam_at"] = _clock[0] - gl.ROAM_CHECK
        gl.wifi_tick()
        roamed = [c[0] for c in WLAN.connects][:1] == ["home"]
        r2_until(lambda: gl.wifi["state"] == "connected", 30, step=500)
        gl.wifi_tick()                   # learning scan through the mesh node
        via = list(gl.wifi["home_aps"])
        gl._wifi_off(w, "sleep")
        WLAN.scan_aps = [("", -45, R2_NEIGH)]
        stranger = gl.home_in_range()
        rv_connect(["home"], [("", -50, R2_OURS)],
                   nets=[dict(n) for n in hid])
        WLAN.scan_aps = [("home", -50, R2_MESH), ("", -48, R2_NEIGH)]
        gl.wifi_tick()
        named = list(gl.wifi["home_aps"])
        check("R2-7: an unlearned node of the hidden home is enough to roam "
              "and is learned through its link", on_hot and roamed
              and via == [R2_MESH, R2_OURS] and stranger is False
              and named == [R2_MESH], "%s %s %r %r %r" % (
                  on_hot, roamed, via, stranger, named))

        # R2-14: a failed hidden roam never blames a learned BSSID of ours
        # (one failed try shut our own router out for an hour)
        gl.wifi_reset()
        gl.wifi["nets"] = [dict(n) for n in hid]
        gl.wifi.update(home_aps=[R2_OURS], blank_rssi=-50,
                       blank_bssid=R2_OURS, roaming=True)
        gl._mark_failed("home", "test")
        ours_ok = gl.wifi["blank_bad"] == [] and gl._blank_home_ok()
        check("R2-14: a failed hidden roam does not blame a learned BSSID",
              ours_ok, repr(gl.wifi["blank_bad"]))

        # R2-b: a blamed stranger's blank AP is skipped when the scan picks
        # the strongest one (it hid a weaker AP that may be ours)
        gl.wifi_reset()
        gl.wifi["nets"] = [dict(n) for n in hid]
        gl.wifi["blank_bad"] = [(R2_NEIGH, _clock[0])]
        WLAN.scan_aps = [("", -40, R2_NEIGH), ("", -60, R2_MESH)]
        gl._scan_known(WLAN(0))
        skip = (gl.wifi["blank_bssid"], gl._blank_home_ok())
        check("R2-b: a blamed blank AP is skipped for the strongest one",
              skip == (R2_MESH, True), repr(skip))

        # R2-10: a network the scans do not list (a hotspot that does not
        # beacon yet, a hidden SSID without *_HIDDEN=1) is tried in the first
        # round after a link loss, a wake or an arrival: the round count ran
        # on, so it waited for every UNLISTED_EVERY-th round (~38 min)
        w = rv_connect(["home"], [("home", -50)])
        r0 = gl.wifi["rounds"]
        w._status = 0
        w._requested = False
        WLAN.ok = {"hot"}
        WLAN.scan_aps = [("neighbour", -60, b"\x09" * 6)]
        WLAN.connects = []
        got = r2_until(lambda: gl.wifi["state"] == "connected", 60)
        tried = [c[0] for c in WLAN.connects]
        gl.wifi.update(state="not found", rounds=3)
        gl.wifi_wake()
        woke = gl.wifi["rounds"]
        gl._wifi_off(w, "sleep")
        gl.wifi.update(rounds=3, wake=True)
        gl.wifi_tick()
        slept = gl.wifi["rounds"]
        check("R2-10: the first round after a link loss / wake tries the "
              "unlisted networks", r0 % gl.UNLISTED_EVERY != 0 and got
              and gl.wifi["ssid"] == "hot" and "hot" in tried and woke == 0
              and slept == 1, "%d %s %r %r %r" % (r0, got, tried, woke, slept))

        # R2-13: no GEO_URL/API_KEY ("no .env") with tracks waiting: away,
        # nothing in range, it searched every backoff / hour (34 rounds in
        # 6 h) for an upload that never happens
        gl.PB_KEEPALIVE = 0
        make_trk(os.path.join(rv_td, "20261031_110000.trk"), 3)
        w = r2_boot("SSID=home\nWPA2=pw\n", [("home", -50)], ["home"])
        for _ in range(30):
            _clock[0] += 1000
            gl.wifi_tick()
            gl.upload_tick()
        idle = (gl.up["state"], gl.wifi["state"])
        WLAN.ok, WLAN.scan_aps = set(), []
        r0 = gl.wifi["rounds"]
        for _ in range(6 * 360):
            _clock[0] += 10000
            gl.wifi_tick()
            gl.upload_tick()
        rounds = gl.wifi["rounds"] - r0
        os.remove(os.path.join(rv_td, "20261031_110000.trk"))
        check("R2-13: 'no .env' with tracks waiting: no rounds while away",
              idle == ("no .env", "sleep") and rounds <= 1,
              "%r rounds=%d" % (idle, rounds))

        # R2-6 / R2-e: a hidden home that boots after the board, default
        # scan keep-alive, WEB on: a keep-alive scan cannot see a hidden
        # home whose BSSIDs are not known, so the failed boot round sleeps
        # WIFI_SLEEP (status page down ~1 h). Now it searches on
        gl.WEB, gl.PB_KEEPALIVE, gl.PB_MODE = True, 5000, "scan"

        def r2_late_router(env_text, blank):
            r2_boot(env_text, [], [])
            gl.WEB, gl.PB_KEEPALIVE, gl.PB_MODE = True, 5000, "scan"
            r2_until(lambda: gl.wifi["state"] != "connecting", 60)
            boot = gl.wifi["state"]
            _clock[0] += 60000
            t0 = _clock[0]
            WLAN.ok = {"home"}
            WLAN.scan_aps = [("" if blank else "home", -50, R2_OURS)]
            ok = r2_until(lambda: gl.wifi["state"] == "connected", 900,
                          pulse=True)
            return boot, ((_clock[0] - t0) // 1000 if ok else None)
        hid_env = "SSID=home\nWPA2=pw\nSSID_HIDDEN=1\n"
        boot_h, back_h = r2_late_router(hid_env, True)
        boot_v, back_v = r2_late_router("SSID=home\nWPA2=pw\n", False)
        check("R2-6: a hidden home up after the board is found within "
              "minutes (scan keep-alive, WEB)", boot_h == "not found"
              and back_h is not None and back_h <= 300
              and boot_v == "sleep" and back_v is not None and back_v <= 120,
              "%s %r %s %r" % (boot_h, back_h, boot_v, back_v))

        # R2-a: the learned BSSIDs are kept in HOME_APS_FILE (written when
        # the list changes, not into MIN_FREE) and read at boot, so the
        # keep-alive wake knows the hidden home before it connected once;
        # a damaged file is no error
        gl.HOME_APS_FILE = r2_haf
        if os.path.exists(r2_haf):
            os.remove(r2_haf)
        gl.WEB = False
        rv_connect(["home"], [("", -50, R2_OURS)],
                   nets=[dict(n) for n in hid])
        gl.wifi_tick()
        try:
            with open(r2_haf) as f:
                saved = f.read()
        except OSError:
            saved = None
        rv_connect(["home"], [("", -50, R2_MESH)],
                   nets=[dict(n) for n in hid])
        gl.free = lambda: gl.MIN_FREE - 1
        gl.wifi_tick()
        gl.free = r2_saved[3]
        try:
            with open(r2_haf) as f:
                low = f.read()
        except OSError:
            low = None
        with open(r2_haf, "wb") as f:
            f.write(b"zz\n515151\n\xff\xfe\xfd\nABCDEFabcdef\n515151515151\n"
                    b"515151515151\n")
        r2_boot(hid_env, [], [])
        loaded = list(gl.wifi["home_aps"])
        r2_boot("SSID=home\nWPA2=pw\n", [], [])
        visible = list(gl.wifi["home_aps"])
        os.remove(r2_haf)
        r2_boot(hid_env, [], [])
        none = list(gl.wifi["home_aps"])
        check("R2-a: learned BSSIDs are kept in a file and read at boot",
              saved == "515151515151\n" and low == saved
              and loaded == [b"\xab\xcd\xef\xab\xcd\xef", R2_OURS]
              and visible == [] and none == [],
              "%r %r %r %r %r" % (saved, low, loaded, visible, none))

        # R2-6 / R2-a / R2-e: rebooted with the hidden home's BSSID on file,
        # router up after the board: a keep-alive scan seeing it wakes the
        # radio at once
        with open(r2_haf, "w") as f:
            f.write("515151515151\n")
        boot_f, back_f = r2_late_router(hid_env, True)
        check("R2-6: with the hidden home's BSSID on file a keep-alive scan "
              "wakes the radio", boot_f == "sleep" and back_f is not None
              and back_f <= 60, "%s %r" % (boot_f, back_f))
    finally:
        (WLAN.assoc_channel, gl.PB_KEEPALIVE, gl.HOME_APS_FILE,
         gl.free) = r2_saved
        if os.path.exists(r2_haf):
            os.remove(r2_haf)
finally:
    (gl.WEB, gl.WEB_AWAY, WLAN.scan_aps, WLAN.ok, gl.PB_MODE,
     gl.PB_CONNECTED) = saved_rv
    gl.wifi_reset()


# Logger / stays: findings 4, 5, 6, 12, 24 and the empty-file flush
rv_dir = tempfile.mkdtemp(prefix="gpxlog-rv29-")
gl.LOG_DIR = rv_dir
gl.WEB = False


def rv_gga(t):
    return rv_nmea("GNGGA,%s.00,4730.0000,N,01900.0000,E,1,12,0.9,100.0,M,"
                   "40.0,M,," % t) + b"\r\n"


def rv_rmc(t):
    return rv_nmea("GNRMC,%s.00,A,4730.0000,N,01900.0000,E,0.1,,181026,,,A"
                   % t) + b"\r\n"


# L-4: a second without a date (GGA only, a warm boot) is skipped
sv_reset()
_uart_rx[:] = []
data = rv_gga("080000")
for t in ("080001", "080002"):
    for _ in range(5):
        data += rv_rmc(t) + rv_gga(t)
data += rv_rmc("080003")
_uart_rx.extend(data)
gl.read_uart()
crash = None
try:
    gl.log_tick()
except Exception as e:
    crash = e
check("L-4: a queued second without a date is skipped, no crash",
      crash is None and all(l.split(",")[4].strip()[:8] == "20261018"
                            for l in sv_lines()), repr(crash))

# L-5: an away stay, home WiFi, no fix for STILL_GAP -> the stay ends
sv_reset()
sv_drift(150)
active0 = sv_active()
for _ in range(40 * 60):
    hb_fix(0, 0, nofix=True)
gl.wifi.update(state="connected", home=True)
for _ in range(60):
    hb_fix(0, 0, nofix=True)
check("L-5: away stay, home without a fix -> the stay ends",
      active0 and not sv_active(), "active=%s" % sv_active())
gl.wifi.update(state="offline", home=False)

# L-5b: the home WiFi coming up just after a stay began (arriving home,
# WiFi not yet connected) is no "home without a fix": the stay began
# with a fix (it had none on record, read as none for STILL_GAP)
sv_reset()
gl.wifi["sleep_home"] = False            # (left over: not a home stay)
hrnd = random.Random(5)
for _ in range(300):
    sv_fix(hrnd.uniform(-8, 8), hrnd.uniform(-8, 8), 1.0)
    if sv_active():
        break
active0, home0 = sv_active(), gl.log["still"]["home"]
gl.st["valid"] = False                   # indoors: no usable second since
gl.wifi.update(state="connected", home=True)
for _ in range(5):
    hb_fix(0, 0, nofix=True, speed=0.0)
kept = sv_active()
for _ in range(gl.STILL_GAP // 1000):
    hb_fix(0, 0, nofix=True, speed=0.0)
check("L-5b: home WiFi just after a stay began keeps the stay",
      active0 and not home0 and kept,
      "active0=%s home0=%s kept=%s" % (active0, home0, kept))
check("L-5b: ... and STILL_GAP without a fix still ends it",
      not sv_active())
gl.wifi.update(state="offline", home=False)

# L-6: a drive whose fix is lost: little stays in RAM, and it is written
sv_reset()
for i in range(10 * 60):
    sv_fix(i * 15.0, 0, 54.0)
for _ in range(60):
    hb_fix(0, 0, nofix=True)
in_ram = len(gl.log["batch"])
for _ in range(gl.HOLD_RELEASE // 1000 + 30):
    hb_fix(0, 0, nofix=True)
check("L-6: after a drive at most a window waits in RAM; later written",
      in_ram <= 70 and len(gl.log["batch"]) == 0, "ram=%d then %d"
      % (in_ram, len(gl.log["batch"])))
# L-6b: an open window that has spread out (a drive) holds nothing: before,
# a flush kept up to a whole window of the drive in RAM
sv_reset()
for i in range(10 * 60 + 40):                # 40 s into a window
    sv_fix(i * 15.0, 0, 54.0)
gl.flush(gl._still_hold())
check("L-6b: a flush during a drive leaves little of it in RAM",
      len(gl.log["batch"]) <= 15, "ram=%d" % len(gl.log["batch"]))

# L-12: the heartbeat comes after the queued seconds, in time order
sv_reset()
sv_drift(150)
sl12 = gl.log["still"]
while gl._age(_clock[0] + 1000, sl12["hb_at"]) < gl.STILL_HEARTBEAT - 3000:
    sv_fix(rnd.uniform(-5, 5), rnd.uniform(-5, 5))
gl.log["seq"] = gl.st["seq"]
gl.st["q"] = []
_clock[0] += 12000                       # a 12 s blocking POST
for k in range(10):                      # meanwhile: driving off, queued
    sec = rv_sec[0]
    rv_sec[0] += 1
    gl.st.update(seq=gl.st["seq"] + 1, valid=True, date="2026-10-18",
                 time="%02d:%02d:%02d" % (8 + sec // 3600, sec // 60 % 60,
                                          sec % 60),
                 lat=47.5, lon=19.0 + (160 + 20 * k) * M_LON, alt=100.0,
                 hdop=1.0, std=None, std_at=None, epochs=5, status="A",
                 fixtype=3, pos_at=_clock[0], hdop_at=_clock[0])
    gl.st["q"].append(gl._snap())
gl.log_tick()
times = [l.split(",")[4].strip() for l in sv_lines()]
check("L-12: no heartbeat before queued seconds (lines in time order)",
      times == sorted(times) and len(set(times)) == len(times),
      repr(times[-12:]))

# L-24: the arrival file could not be closed: the stay still rotates it
sv_reset()
real_close = gl._close_file
gl._close_file = lambda: False
try:
    sv_drift(150)
finally:
    gl._close_file = real_close
check("L-24: a stay whose arrival close failed gets a file_at",
      sv_active() and gl.log["file"] is not None
      and gl.log["still"]["file_at"] is not None)

# no empty file from a flush with nothing to write
sv_reset()
gl.start_file("20261018_090000")
gl.flush()
check("FLUSH: nothing to write creates no file",
      not os.path.exists(os.path.join(rv_dir, "20261018_090000.trk")))


# Upload / web: findings 10, 11, 13-17 and the upload/web improvements
up_td = tempfile.mkdtemp(prefix="rev29up-")
online(up_td)
gl.up["url"], gl.up["key"] = "https://d.example", "k"

# U-10: an unreadable oldest track neither blocks the rest nor stays
bad_p = os.path.join(up_td, "20261001_100000.trk")
ok_p = os.path.join(up_td, "20261002_100000.trk")
make_trk(bad_p, 3)
make_trk(ok_p, 3)
real_rb = gl.read_batch


def rb_fail(path, off=0):
    if path == bad_p:
        raise OSError(5)
    return real_rb(path, off)
gl.read_batch = rb_fail
_posts[:] = []
_post_code[0] = 201
try:
    for _ in range(gl.READ_ERR_MAX + 4):
        gl.up["next_at"] = _clock[0]
        for p_ in list(gl.up["hold"]):
            gl.up["hold"][p_] = _clock[0] - 1    # holds over
        gl.upload_tick()
        _clock[0] += 1000
finally:
    gl.read_batch = real_rb
check("U-10: an unreadable track is held, the next uploads, then .err",
      len(_posts) >= 1 and not os.path.exists(ok_p)
      and os.path.exists(bad_p + ".err") and not os.path.exists(bad_p),
      "posts=%d %r" % (len(_posts), os.listdir(up_td)))
os.remove(bad_p + ".err")

# POS-ERR: a batch with nothing to send whose .pos write fails is a flash
# error, not an unreadable track: READ_ERR_MAX of them must not park it
pe_p = os.path.join(up_td, "20261002_100000.trk")
make_trk(pe_p, gl.UPLOAD_BATCH + 10,
         times=["2026-10-05T00:00:%02dX" % (i % 60)
                for i in range(gl.UPLOAD_BATCH + 10)])
real_wo = gl.write_offset
gl.write_offset = lambda path, off: (_ for _ in ()).throw(OSError(28))
_posts[:] = []
try:
    for _ in range(gl.READ_ERR_MAX + 2):
        gl.up["next_at"] = _clock[0]
        gl.up["hold"] = {}
        gl.upload_tick()
        _clock[0] += 1000
finally:
    gl.write_offset = real_wo
check("POS-ERR: a failing .pos write is 'fs err', the track is not parked",
      os.path.exists(pe_p) and not os.path.exists(pe_p + ".err")
      and gl.up["state"] == "fs err" and not _posts,
      "state %s %r" % (gl.up["state"], os.listdir(up_td)))
for x_ in (pe_p, pe_p + ".err"):
    if os.path.exists(x_):
        os.remove(x_)

# U-14: the quarantine copy lands but the offset write fails: no crash
q_p = os.path.join(up_td, "20261003_100000.trk")
make_trk(q_p, gl.UPLOAD_BATCH + 5)      # not EOF: the .pos is written
_post_code[0] = 422
real_wo = gl.write_offset
gl.write_offset = lambda path, off: (_ for _ in ()).throw(OSError(28))
crash = None
try:
    pump()
except Exception as e:
    crash = e
finally:
    gl.write_offset = real_wo
    _post_code[0] = 201
check("U-14: quarantine with a failing offset write: fs err, no crash",
      crash is None and gl.up["state"] == "fs err", repr(crash))
for n_ in os.listdir(up_td):
    os.remove(os.path.join(up_td, n_))
online(up_td)
gl.up["url"], gl.up["key"] = "https://d.example", "k"

# U-15: a long .bad drains past MAX_BAD_RETRY good batches in one boot
b_p = os.path.join(up_td, "20261004_100000.trk.bad")
make_trk(b_p, gl.UPLOAD_BATCH * (gl.MAX_BAD_RETRY + 3))
_posts[:] = []
for _ in range(gl.MAX_BAD_RETRY + 6):
    pump()
    _clock[0] += 1000
check("U-15: a .bad keeps draining after MAX_BAD_RETRY good batches",
      len(_posts) >= gl.MAX_BAD_RETRY + 3 and not os.path.exists(b_p),
      "posts=%d" % len(_posts))

# FAIL4-BAD: rejected .bad batches are not in fail4 (reset only by a 2xx):
# a normal track's 422 batch afterwards still goes to quarantine
online(up_td)
gl.up["url"], gl.up["key"] = "https://d.example", "k"
fb_p = os.path.join(up_td, "20261004_110000.trk.bad")
make_trk(fb_p, 3)
_post_code[0] = 422
for _ in range(gl.MAX_GLOBAL_PAYLOAD + 1):
    gl.up["hold"] = {}
    pump()
    _clock[0] += 1000
bad_posts = len(_posts)
fn_p = os.path.join(up_td, "20261004_120000.trk")
make_trk(fn_p, 3)
pump()
_post_code[0] = 201
check("FAIL4-BAD: after rejected .bad batches a normal 422 is quarantined",
      bad_posts == gl.MAX_GLOBAL_PAYLOAD + 1
      and gl.up["state"] == "quarantine" and not os.path.exists(fn_p)
      and pts_in(fn_p + ".bad") == 3,
      "bad posts %d state %s fail4 %r" % (bad_posts, gl.up["state"],
                                          gl.up.get("fail4")))
for n_ in os.listdir(up_td):
    os.remove(os.path.join(up_td, n_))
online(up_td)
gl.up["url"], gl.up["key"] = "https://d.example", "k"

# U-16: at EOF the track goes without writing a .pos first (full flash)
e_p = os.path.join(up_td, "20261005_100000.trk")
make_trk(e_p, 3)
gl.write_offset = lambda path, off: (_ for _ in ()).throw(OSError(28))
try:
    pump()
finally:
    gl.write_offset = real_wo
check("U-16: the last batch deletes the track even if .pos cannot be written",
      not os.path.exists(e_p), repr(os.listdir(up_td)))

# U-17 / U-11: own POST: no redirects, chunked body and polled wait
class RvSock:
    def __init__(self, resp, polls=3):
        self.resp, self.polls, self.out, self.closed = resp, polls, b"", False
        self.rd = 0
        self.timeout = None

    def settimeout(self, t):
        self.timeout = t

    def connect(self, addr):
        self.addr = addr

    def write(self, b):
        b = bytes(b)
        self.out += b
        return len(b)

    def readline(self, n=-1):
        self.limits = getattr(self, "limits", []) + [n]
        if self.rd >= len(self.resp) and getattr(self, "fail", None):
            raise self.fail                  # reset / closed after the head
        i = self.resp.find(b"\n", self.rd)
        i = len(self.resp) if i < 0 else i + 1
        if n is not None and n >= 0:
            i = min(i, self.rd + n)
        line, self.rd = self.resp[self.rd:i], i
        return line

    def close(self):
        self.closed = True


class RvPoll:
    def __init__(self, sock):
        self.sock = sock

    def register(self, s, ev):
        pass

    def poll(self, ms):
        _clock[0] += ms
        if self.sock.polls > 0:
            self.sock.polls -= 1
            return []
        return [(self.sock, 1)]


class RvTls:
    PROTOCOL_TLS_CLIENT, CERT_NONE, CERT_REQUIRED = 2, 0, 2
    made = []

    class SSLContext:
        def __init__(self, proto):
            self.verify_mode, self.ca = None, None
            RvTls.made.append(self)

        def load_verify_locations(self, cadata=None):
            self.ca = cadata

        def wrap_socket(self, s, server_hostname=None):
            self.host = server_hostname
            return s


def tls_ctx_reset():
    """Forget the SSLContext made once a boot (tests swap tls and CA_FILE)."""
    c = getattr(gl, "_tls_ctx", None)
    if c:
        c[0] = None


rv_sock = [None]
fake_sockmod = types.ModuleType("socket")
fake_sockmod.SOCK_STREAM = 1
fake_sockmod.getaddrinfo = lambda h, p, a=0, b=0: [(2, 1, 6, "", (h, p))]
fake_sockmod.socket = lambda *a: rv_sock[0]
fake_sel = types.ModuleType("select")
fake_sel.POLLIN = 1
fake_sel.poll = lambda: RvPoll(rv_sock[0])
real_mods = (gl.socket, gl.select, sys.modules.get("tls"))
gl.socket, gl.select = fake_sockmod, fake_sel
sys.modules["tls"] = RvTls
tls_ctx_reset()                          # up_reset() made one with the mock
uart_reads = [0]
real_ru = gl.read_uart
gl.read_uart = lambda: uart_reads.__setitem__(0, uart_reads[0] + 1)
try:
    rv_sock[0] = RvSock(b"HTTP/1.1 201 Created\r\nRetry-After: 7\r\n\r\n",
                        polls=3)
    body = b"x" * (gl.HTTP_CHUNK * 3 + 10)
    r = gl._raw_post("https://d.example:8443/sub", body,
                     {"Authorization": "Bearer k"}, 15)
    sent = rv_sock[0].out
    ok_post = (r.status_code == 201 and r.headers.get("Retry-After") == "7"
               and sent.startswith(b"POST /sub HTTP/1.0\r\nHost: d.example\r\n")
               and b"Content-Length: %d\r\n" % len(body) in sent
               and sent.endswith(body) and rv_sock[0].closed
               and rv_sock[0].addr == ("d.example", 8443)
               and RvTls.made[-1].verify_mode == RvTls.CERT_NONE
               and uart_reads[0] >= 4 + 3)
    rv_sock[0] = RvSock(b"HTTP/1.1 301 Moved\r\nLocation: https://x/\r\n\r\n",
                        polls=0)
    r301 = gl._raw_post("https://d.example", b"{}", {}, 15)
    one_req = rv_sock[0].out.count(b"POST ") == 1
    ca_path = os.path.join(up_td, "ca.der")
    with open(ca_path, "wb") as f:
        f.write(b"\x30\x82der")
    gl.CA_FILE = ca_path
    tls_ctx_reset()                      # the context is made once a boot
    rv_sock[0] = RvSock(b"HTTP/1.1 201 Created\r\n\r\n", polls=0)
    gl._raw_post("https://d.example", b"{}", {}, 15)
    ca_ctx = RvTls.made[-1]
    # TLS-CTX: later POSTs reuse that context, CA_FILE is not read again
    made0 = len(RvTls.made)
    ca_reads = []
    real_open_ = builtins.open

    def _ca_open(name, *a, **k):
        if name == ca_path:
            ca_reads.append(name)
        return real_open_(name, *a, **k)
    builtins.open = _ca_open
    try:
        for _ in range(3):
            rv_sock[0] = RvSock(b"HTTP/1.1 201 Created\r\n\r\n", polls=0)
            gl._raw_post("https://d.example", b"{}", {}, 15)
    finally:
        builtins.open = real_open_
    ctx_reused = (len(RvTls.made) == made0 and not ca_reads
                  and RvTls.made[-1] is ca_ctx)
    # TLS-CTX: a context that failed to be made is made again next time
    tls_ctx_reset()
    real_ctx_cls = RvTls.SSLContext

    class _NoMemCtx(object):
        def __init__(self, proto):
            raise MemoryError("ctx")
    RvTls.SSLContext = _NoMemCtx
    try:
        rv_sock[0] = RvSock(b"HTTP/1.1 201 Created\r\n\r\n", polls=0)
        try:
            gl._raw_post("https://d.example", b"{}", {}, 15)
            ctx_fail = None
        except MemoryError as e:
            ctx_fail = e
    finally:
        RvTls.SSLContext = real_ctx_cls
    rv_sock[0] = RvSock(b"HTTP/1.1 201 Created\r\n\r\n", polls=0)
    r_again = gl._raw_post("https://d.example", b"{}", {}, 15)
    ctx_retry = (ctx_fail is not None and r_again.status_code == 201
                 and getattr(gl, "_tls_ctx", [None])[0] is RvTls.made[-1])
    rv_sock[0] = RvSock(b"", polls=10 ** 6)
    t0 = _clock[0]
    try:
        gl._raw_post("https://d.example", b"{}", {}, 15)
        to = None
    except OSError as e:
        to = e
finally:
    gl.socket, gl.select = real_mods[0], real_mods[1]
    if real_mods[2] is None:
        sys.modules.pop("tls", None)
    else:
        sys.modules["tls"] = real_mods[2]
    gl.read_uart = real_ru
    gl.CA_FILE = "/ca.der"
    tls_ctx_reset()
check("U-11: POST body in chunks, UART read while writing and waiting",
      ok_post, repr(rv_sock[0].out[:80]))
check("U-17: a 301 is returned, not followed (no GET, no Bearer elsewhere)",
      r301.status_code == 301 and one_req)
check("TLS: with CA_FILE on flash the server is verified",
      ca_ctx.verify_mode == RvTls.CERT_REQUIRED and ca_ctx.ca == b"\x30\x82der")
check("TLS-CTX: one SSLContext for every POST, CA_FILE read once",
      ctx_reused, "made %d->%d reads %r" % (made0, len(RvTls.made), ca_reads))
check("TLS-CTX: a context that failed to be made is made on the next POST",
      ctx_retry, repr(ctx_fail))
check("U-11: no answer within HTTP_WAIT -> timeout, UART kept up",
      to is not None and _clock[0] - t0 >= gl.HTTP_WAIT, repr(to))

# HEAD-ERR: once the status line is in, an error in the header loop keeps
# the 2xx (a reset there resent the batch: a duplicate); bounded reads
gl.socket, gl.select = fake_sockmod, fake_sel
sys.modules["tls"] = RvTls
try:
    head_res = []
    for fail_ in (OSError(104), OSError(110), OSError(9)):
        rv_sock[0] = RvSock(b"HTTP/1.1 201 Created\r\nServer: x\r\n",
                            polls=0)
        rv_sock[0].fail = fail_
        try:
            head_res.append(gl._raw_post("https://d.example", b"{}", {},
                                         15).status_code)
        except OSError as e:
            head_res.append(e)
    # a header longer than a read, then one cut right before its CRLF:
    # the pieces are neither headers nor the end of the head
    long_v = b"a" * 400
    cut = b"X-Cut: " + b"b" * (256 - 7)
    rv_sock[0] = RvSock(b"HTTP/1.1 429 Too Many\r\nX-Long: " + long_v
                        + b"\r\n" + cut + b"\r\nRetry-After: 9\r\n\r\n",
                        polls=0)
    r_long = gl._raw_post("https://d.example", b"{}", {}, 15)
    limits = rv_sock[0].limits
finally:
    gl.socket, gl.select = real_mods[0], real_mods[1]
    if real_mods[2] is None:
        sys.modules.pop("tls", None)
    else:
        sys.modules["tls"] = real_mods[2]
    tls_ctx_reset()
check("HEAD-ERR: reset/timeout/close in the headers keeps the 201",
      head_res == [201, 201, 201], repr(head_res))
check("HEAD-ERR: answer lines read in bounded pieces, Retry-After still found",
      r_long.status_code == 429 and r_long.headers.get("Retry-After") == "9"
      and limits and all(0 < n_ <= 256 for n_ in limits),
      "%r %r" % (r_long.headers, limits[:4]))


def ctx_now():
    """(verify_mode, ca) of the SSLContext kept, or None."""
    c = getattr(gl, "_tls_ctx", [None])[0]
    return None if c is None else (c.verify_mode, c.ca)


# TLS-CTX-BOOT: up_reset() makes the SSLContext at boot, before the WiFi
# starts (made at the first POST it could grow the heap and pin that area)
n_made0 = len(_TlsCtx.made)
tls_ctx_reset()
online(up_td)
boot_ctx = getattr(gl, "_tls_ctx", [None])[0]
n_made1 = len(_TlsCtx.made)
gl.socket, gl.select = fake_sockmod, fake_sel
try:
    rv_sock[0] = RvSock(b"HTTP/1.1 201 Created\r\n\r\n", polls=0)
    gl._raw_post("https://d.example", b"{}", {}, 15)
finally:
    gl.socket, gl.select = real_mods[0], real_mods[1]
import inspect
run_src = inspect.getsource(gl.run)
check("TLS-CTX-BOOT: up_reset() makes the SSLContext before the WiFi; the "
      "POST uses it", n_made1 == n_made0 + 1 and boot_ctx is not None
      and boot_ctx is _TlsCtx.made[-1] and len(_TlsCtx.made) == n_made1
      and run_src.index("up_reset()") < run_src.index("wifi_start()"),
      "%d %d %d" % (n_made0, n_made1, len(_TlsCtx.made)))

# TLS-CA: a CA_FILE that cannot be read (not ENOENT) or is empty: no POST
# ("ca err", a diag line, a fixed retry), never an unverified context;
# TLS-CA-NEW: a CA_FILE added, replaced (other size) or removed is picked
# up at the next POST, no reboot
ca_td = tempfile.mkdtemp(prefix="ca-")
ca_p = os.path.join(ca_td, "ca.der")
os.mkdir(ca_p)                           # open() fails: EISDIR, not ENOENT
online(up_td)
gl.up["url"], gl.up["key"] = "https://d.example", "k"
gl.CA_FILE = ca_p
tls_ctx_reset()
_posts[:] = []
diag_clear()
gl.DIAG = 60000
try:
    make_trk(os.path.join(up_td, "20261009_100000.trk"), 3)
    pump()
    ca_r1 = (len(_posts), gl.up["state"], gl.up["next_at"] - _clock[0],
             ctx_now())
    os.rmdir(ca_p)
    open(ca_p, "wb").close()             # empty: a cut copy, not "no CA"
    pump()
    ca_r2 = (len(_posts), gl.up["state"], ctx_now())
    with open(ca_p, "wb") as f:
        f.write(b"\x30\x82der")
    pump()
    ca_r3 = (len(_posts), gl.up["state"], ctx_now())
    with open(ca_p, "wb") as f:
        f.write(b"\x30\x82other der")
    make_trk(os.path.join(up_td, "20261009_110000.trk"), 3)
    pump()
    ca_r4 = (len(_posts), ctx_now())
    os.remove(ca_p)
    make_trk(os.path.join(up_td, "20261009_120000.trk"), 3)
    pump()
    ca_r5 = (len(_posts), ctx_now())
finally:
    gl.CA_FILE = "/ca.der"
    tls_ctx_reset()
    gl.DIAG = 0
    shutil.rmtree(ca_td, ignore_errors=True)
ca_diag = [l for l in diag_lines() if l.startswith("upload ca err")]
diag_clear()
check("TLS-CA: unreadable or empty CA_FILE -> no POST, 'ca err', retry after "
      "LOWIDF_RETRY, no unverified context",
      ca_r1 == (0, "ca err", gl.LOWIDF_RETRY, None)
      and ca_r2 == (0, "ca err", None) and len(ca_diag) == 2,
      "%r %r %r" % (ca_r1, ca_r2, ca_diag))
check("TLS-CA-NEW: a readable CA_FILE verifies; replaced or removed it is "
      "read again without a reboot",
      ca_r3 == (1, "ok", (2, b"\x30\x82der"))
      and ca_r4 == (2, (2, b"\x30\x82other der")) and ca_r5 == (3, (0, None)),
      "%r %r %r" % (ca_r3, ca_r4, ca_r5))

# TLS-CA-OOM: modtls reports a failed allocation parsing the CA as
# ValueError("invalid cert"): out of memory only while the context is made
# and the IDF heap is low; else a bad file ("ca err", not in the ENOMEM run)
ca_td = tempfile.mkdtemp(prefix="ca-")
ca_p = os.path.join(ca_td, "ca.der")
with open(ca_p, "wb") as f:
    f.write(b"\x30\x82der")


class _BadCaTls(object):
    PROTOCOL_TLS_CLIENT, CERT_NONE, CERT_REQUIRED = 2, 0, 2

    class SSLContext(_TlsCtx):
        def load_verify_locations(self, cadata=None):
            raise ValueError("invalid cert")


online(up_td)
saved_tls = sys.modules.get("tls")
sys.modules["tls"] = _BadCaTls
gl.CA_FILE = ca_p
tls_ctx_reset()
real_idf = gl.idf_heap
oom_res = []
try:
    tr = getattr(gl, "_tls_ready", None)
    for idf in ((30, 20), (157, 126)):
        gl.idf_heap = lambda idf=idf: idf
        gl.up["enomem"] = 2
        ok_ = tr(_clock[0]) if tr else None
        oom_res.append((ok_, gl.up["state"], gl.up["enomem"], ctx_now()))
    gl._post_phase[0] = "http"
    gl.idf_heap = lambda: (30, 20)
    later = gl._enomem(ValueError("invalid cert"))
finally:
    gl._post_phase[0] = None
    gl.idf_heap = real_idf
    sys.modules["tls"] = saved_tls
    gl.CA_FILE = "/ca.der"
    tls_ctx_reset()
    shutil.rmtree(ca_td, ignore_errors=True)
check("TLS-CA-OOM: ValueError making the context counts as out of memory "
      "only with the IDF heap low",
      oom_res == [(False, "low mem", 3, None), (False, "ca err", 0, None)]
      and later is False, "%r %r" % (oom_res, later))

# RTC: mbedTLS checks certificate dates against the board clock (2000-01-01
# at boot): set it from a valid GPS fix before a POST, once, again only when
# it drifted more than RTC_DRIFT s; no fix or a date outside 2020-2087: no
import datetime as _dt
saved_st = dict((k, gl.st.get(k)) for k in ("valid", "date", "time", "lat",
                                              "pos_at"))
rtc_res = []
rs = getattr(gl, "_rtc_sync", None)
try:
    def _fix(date, time_, valid=True):
        gl.st.update(valid=valid, date=date, time=time_, lat=47.1,
                     pos_at=_clock[0])
    RTC.dt, RTC.sets[:] = (2000, 1, 1, 5, 0, 0, 0, 0), []
    if hasattr(gl, "_rtc_set"):
        gl._rtc_set[0] = False
    wd = _dt.date(2026, 10, 31).weekday()
    _fix("2026-10-31", "12:00:00")
    if rs:
        rs(_clock[0])
        rs(_clock[0])                    # set already: nothing
        RTC.dt = (2026, 10, 31, wd, 12, 0, 3, 0)
        rs(_clock[0])                    # 3 s off: kept
        RTC.dt = (2026, 10, 31, wd, 11, 59, 50, 0)
        rs(_clock[0])                    # 10 s off: set again
    rtc_res.append(RTC.sets[:])
    RTC.sets[:] = []
    if hasattr(gl, "_rtc_set"):
        gl._rtc_set[0] = False
    for args in (("2026-10-31", "12:00:00", False), ("2099-01-01", "00:00:00"),
                 ("2019-12-31", "23:59:59"), (None, None)):
        _fix(*args)
        if rs:
            rs(_clock[0])
    rtc_res.append(RTC.sets[:])
    # through upload_tick: the clock is set before the POST
    online(up_td)
    gl.up["url"], gl.up["key"] = "https://d.example", "k"
    make_trk(os.path.join(up_td, "20261031_120000.trk"), 3)
    RTC.dt, RTC.sets[:] = (2000, 1, 1, 5, 0, 0, 0, 0), []
    if hasattr(gl, "_rtc_set"):
        gl._rtc_set[0] = False
    _fix("2026-10-31", "12:00:00")
    _posts[:] = []
    diag_clear()
    gl.DIAG = 60000
    pump()
    rtc_res.append((RTC.sets[:], len(_posts),
                    [l for l in diag_lines() if l.startswith("clock set")]))
finally:
    gl.st.update(saved_st)
    gl.DIAG = 0
    diag_clear()
check("RTC: the board clock is set from the GPS fix once, again past "
      "RTC_DRIFT", rtc_res[0] == [(2026, 10, 31, wd, 12, 0, 0, 0)] * 2,
      repr(rtc_res[0]))
check("RTC: no valid fix or a date outside 2020-2087 leaves the clock",
      rtc_res[1] == [], repr(rtc_res[1]))
check("RTC: upload_tick sets the clock before the POST (one diag line)",
      rtc_res[2] == ([(2026, 10, 31, wd, 12, 0, 0, 0)], 1,
                     ["clock set 2026-10-31T12:00:00Z"]), repr(rtc_res[2]))

# RTC-PVT (new feature): NAV-PVT time sets the clock when it is valid, fully
# resolved and tAcc < PVT_MAX_TACC, a position fix or not (indoors at home)


def rtc_pvt(**kw):
    gl.reset()
    RTC.dt, RTC.sets[:] = (2000, 1, 1, 5, 0, 0, 0, 0), []
    gl._rtc_set[0] = False
    pvt_second((2026, 10, 31, 12, 0, 0), flags=0, fix=0, **kw)
    pvt_second((2026, 10, 31, 12, 0, 1), n=1, flags=0, fix=0, **kw)
    gl._rtc_sync(_clock[0])
    return RTC.sets[:]


rp_ok = rtc_pvt()
rp_tacc = rtc_pvt(tacc=gl.PVT_MAX_TACC + 1)
rp_unres = rtc_pvt(valid=0x03)
rp_nodate = rtc_pvt(valid=0x06)
gl.reset()
check("RTC-PVT: valid, fully resolved NAV-PVT time sets the clock, no fix "
      "needed", rp_ok == [(2026, 10, 31, wd, 12, 0, 0, 0)], repr(rp_ok))
check("RTC-PVT: tAcc too large, not fully resolved or no valid date: not",
      rp_tacc == [] and rp_unres == [] and rp_nodate == [],
      repr((rp_tacc, rp_unres, rp_nodate)))

# TLS out of memory TLS_REBOOT_N times in a row at home: a clean reboot
for n_ in os.listdir(up_td):
    os.remove(os.path.join(up_td, n_))
online(up_td)
gl.up["url"], gl.up["key"] = "https://d.example", "k"
gl.wifi["home"] = True
make_trk(os.path.join(up_td, "20261006_100000.trk"), 3)
_resets[:] = []
for _ in range(gl.TLS_REBOOT_N):
    _post_raise[0] = OSError(-0x7F00)
    try:
        pump()
    except SystemExit:
        break
    _clock[0] += 1000
check("TLS: ENOMEM TLS_REBOOT_N times at home -> one controlled reboot",
      len(_resets) == 1 and gl.up["enomem"] >= gl.TLS_REBOOT_N,
      "resets=%d enomem=%d" % (len(_resets), gl.up["enomem"]))
_post_raise[0] = None
gl.wifi["home"] = False

# ENOMEM-RUN: "in a row" means consecutive: another failure ends the run
for n_ in os.listdir(up_td):
    os.remove(os.path.join(up_td, n_))
online(up_td)
gl.up["url"], gl.up["key"] = "https://d.example", "k"
gl.wifi["home"] = True
make_trk(os.path.join(up_td, "20261006_110000.trk"), 3)
_resets[:] = []
seq = ([OSError(12)] * (gl.TLS_REBOOT_N - 1) + [OSError(113)]
       + [OSError(12)] * (gl.TLS_REBOOT_N - 1))
try:
    for e_ in seq:
        _post_raise[0] = e_
        try:
            pump()
        except SystemExit:
            break
        _clock[0] += 1000
finally:
    _post_raise[0] = None
    gl.wifi["home"] = False
check("ENOMEM-RUN: ENOMEMs split by a network error never reboot",
      not _resets and gl.up["enomem"] == gl.TLS_REBOOT_N - 1,
      "resets=%d enomem=%d" % (len(_resets), gl.up["enomem"]))

# ENOMEM-LOWIDF: a low-idf episode away does not arm the ENOMEM reboot:
# the first ordinary network error at home is just a retry
online(up_td)
gl.up["url"], gl.up["key"] = "https://d.example", "k"
make_trk(os.path.join(up_td, "20261006_120000.trk"), 3)
real_idf_ = gl.idf_heap
_resets[:] = []
try:
    gl.wifi["home"] = False
    gl.idf_heap = lambda: (30, 20)
    pump()
    lowidf_away = gl.up["state"]
    gl.idf_heap = lambda: (157, 126)
    gl.wifi["home"] = True
    _post_raise[0] = OSError(113)
    try:
        pump()
    except SystemExit:
        pass
finally:
    gl.idf_heap = real_idf_
    _post_raise[0] = None
    gl.wifi["home"] = False
check("ENOMEM-LOWIDF: low idf away, then a net error at home: no reboot",
      lowidf_away == "low idf" and not _resets
      and gl.up["state"] == "err net", "resets=%d state=%s" % (
          len(_resets), gl.up["state"]))

# ENOMEM-CODES: mbedTLS ALLOC_FAILED as modtls raises them
gl._post_phase = getattr(gl, "_post_phase", [None])
gl._post_phase[0] = "http"
codes_ok = all(gl._enomem(OSError(c)) for c in (
    12, -0x7F00, -0x2880, -0x4D80, -0x3F80, -0x3D10, -(0x2180 + 0x6A)))
codes_no = not any(gl._enomem(OSError(c)) for c in (
    16, 106, 113, 104, 110, -0x7200, -0x2700))
check("ENOMEM-CODES: X509/ECP/PK and high+low ALLOC_FAILED codes count",
      codes_ok and codes_no)
gl._post_phase[0] = None
# ... and 16 (MPI_ALLOC_FAILED after modtls, also EBUSY) only from the
# TLS handshake
class RvTls16(RvTls):
    class SSLContext(RvTls.SSLContext):
        def wrap_socket(self, s, server_hostname=None):
            raise OSError(16)


class RvSock16(RvSock):
    def connect(self, addr):
        raise OSError(16)


online(up_td)
gl.up["url"], gl.up["key"] = "https://d.example", "k"
gl.socket, gl.select = fake_sockmod, fake_sel
real_hp = gl.HTTP_POST
gl.HTTP_POST = gl._raw_post
saved_tls = sys.modules.get("tls")
try:
    sys.modules["tls"] = RvTls16
    tls_ctx_reset()
    rv_sock[0] = RvSock(b"", polls=0)
    gl._http_post(10)
    in_tls = gl.up["enomem"]
    rv_sock[0] = RvSock16(b"", polls=0)
    gl._http_post(10)
    in_connect = gl.up["enomem"]
finally:
    gl.HTTP_POST = real_hp
    gl.socket, gl.select = real_mods[0], real_mods[1]
    if saved_tls is None:
        sys.modules.pop("tls", None)
    else:
        sys.modules["tls"] = saved_tls
    tls_ctx_reset()
check("ENOMEM-CODES: 16 counts from the TLS handshake, not from connect",
      in_tls == 1 and in_connect == 0, "%r %r" % (in_tls, in_connect))

# U-13: a request head over 512 B is read whole before the reply
class RvConn:
    def __init__(self, req):
        self.req, self.sent, self.closed = req, b"", False

    def settimeout(self, t):
        pass

    def recv(self, n):
        b, self.req = self.req[:n], self.req[n:]
        return b

    def sendall(self, b):
        self.sent += bytes(b)

    def close(self):
        self.closed = True


long_req = (b"GET /status.json HTTP/1.1\r\nHost: x\r\nCookie: "
            + b"a" * 900 + b"\r\n\r\n")
c = RvConn(long_req)
got = gl._web_read(c)
check("U-13: the whole request head is read (900 B cookie)",
      got == long_req and c.req == b"", "%d" % len(got))
gl.reset()
c = RvConn(b"")
gl._web_reply(c, b"GET /status.json HTTP/1.1")
head, _, jb = c.sent.partition(b"\r\n\r\n")
try:
    jd = json.loads(jb)
except ValueError:
    jd = None
check("WEB: the streamed JSON is valid; the closed connection ends it",
      jd is not None and b"Content-Length" not in head
      and b"Connection: close" in head
      and "bad_files" in jd and "satellites" in jd, repr(head[:200]))
sends = []


class RvConnN(RvConn):
    def sendall(self, b):
        sends.append(len(b))
        RvConn.sendall(self, b)
c = RvConnN(b"")
gl._web_reply(c, b"GET / HTTP/1.1")
check("WEB: the reply goes out in ~WEB_CHUNK sends, not one per piece",
      2 <= len(sends) <= 2 + len(c.sent) // gl.WEB_CHUNK
      and all(n <= gl.WEB_CHUNK + 600 for n in sends[1:]), repr(sends))
gl._fl[:] = [None, None]
calls = []
real_statvfs = os.statvfs
os.statvfs = lambda p: calls.append(p) or real_statvfs(up_td)
try:
    gl._flash_kb()
    gl._flash_kb()
    _clock[0] += 31000
    gl._flash_kb()
finally:
    os.statvfs = real_statvfs
check("WEB: free flash is read at most every 30 s", len(calls) == 2,
      repr(calls))

# WEB-PIECES: no multi-KB piece or buffer on the web path (the Python heap's
# largest free block can be ~2 KB): the diag tail goes out a line a piece,
# read line by line, and a piece over WEB_CHUNK is sent in slices of itself
wd_td = tempfile.mkdtemp(prefix="webdiag-")
saved_diag_file = gl.DIAG_FILE
gl.DIAG_FILE = os.path.join(wd_td, "diag.log")
with open(gl.DIAG_FILE, "w") as f:
    for i in range(60):
        f.write("up %d utc - nmea 1/0 %s\n" % (i, "x" * 140))
whole_reads = []


class LinesOnly(object):
    def __init__(self, f):
        self.f = f

    def seek(self, *a):
        return self.f.seek(*a)

    def readline(self):
        return self.f.readline()

    def read(self, *a):
        whole_reads.append(a)
        return self.f.read(*a)

    def close(self):
        self.f.close()

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.f.close()


real_open = open
gl.open = lambda p, *a, **k: LinesOnly(real_open(p, *a, **k))
try:
    pieces = list(gl._web_json_parts())
finally:
    del gl.open
    gl.DIAG_FILE = saved_diag_file
wd = json.loads("".join(pieces))
check("WEB-PIECES: diag lines are pieces of their own, read in small blocks",
      wd["diag"] == ["up %d utc - nmea 1/0 %s" % (i, "x" * 140)
                     for i in range(45, 60)]
      and max(len(p) for p in pieces) < 400
      and all(r and r[0] <= 1024 for r in whole_reads),
      "%d %r" % (max(len(p) for p in pieces), whole_reads))
sends = []
kinds = []


class RvConnK(RvConnN):
    def sendall(self, b):
        kinds.append(type(b).__name__)
        RvConnN.sendall(self, b)
c = RvConnK(b"")
gl._web_send(c, "200 OK", "text/plain", ("a" * 10, "b" * 3000, "c" * 10))
check("WEB-PIECES: a piece over WEB_CHUNK goes out in WEB_CHUNK memoryview "
      "slices; nothing over WEB_CHUNK is sent",
      c.sent.partition(b"\r\n\r\n")[2] == b"a" * 10 + b"b" * 3000 + b"c" * 10
      and sends[1:] == [10, 1024, 1024, 952, 10]
      and set(kinds[1:]) == {"memoryview"}, "%r %r" % (sends, kinds))

# WEB-FILES: the file counts of the page are listed at most every 10 s
# (pending() and pending_bad() listed, sorted and built paths twice a poll)
fc_td = tempfile.mkdtemp(prefix="webfiles-")
for n in ("20261001_100000.trk", "20261001_110000.gpx",
          "20261001_090000.trk.bad", "20261001_100000.trk.pos",
          "20261001_120000.trk"):
    open(os.path.join(fc_td, n), "w").close()
saved_log_dir, saved_log_file = gl.LOG_DIR, gl.log["file"]
gl.LOG_DIR = fc_td
gl.log["file"] = fc_td + "/20261001_120000.trk"   # being written
gl._fc[2] = None
listings = []
real_listdir = os.listdir
os.listdir = lambda p: listings.append(p) or real_listdir(p)
try:
    r1 = dict(gl._web_rows())
    r2 = dict(gl._web_rows())
    open(os.path.join(fc_td, "20261001_130000.trk"), "w").close()
    _clock[0] += 11000
    r3 = dict(gl._web_rows())
finally:
    os.listdir = real_listdir
    gl.LOG_DIR, gl.log["file"] = saved_log_dir, saved_log_file
check("WEB-FILES: files / bad files counted at most every 10 s",
      len(listings) == 2 and (r1["files"], r1["bad_files"]) == (2, 1)
      and (r2["files"], r3["files"], r3["bad_files"]) == (2, 3, 1),
      "%r %r %r %r" % (listings, (r1["files"], r1["bad_files"]),
                       r2["files"], r3["files"]))


# trk2gpx: .bad/.err files, damaged bytes, non-digit times
t_td = tempfile.mkdtemp(prefix="rev29t-")
t_bad = os.path.join(t_td, "20261029_101500.trk.bad")
with open(t_bad, "wb") as f:
    f.write(b"#TRK1\n47.5,19.0,100.0,0.9,20261029T101500Z\n"
            b"47.5,19.0,,,2026102XT101501Z\n"
            b"47.6,19.\xff1,,,20261029T101502Z\n#END\n")
saved_out = sys.stdout
sys.stdout = io.StringIO()
try:
    rc = trk2gpx.main(["trk2gpx.py", t_bad])
    gpx_out = sys.stdout.getvalue()
finally:
    sys.stdout = saved_out
check("trk2gpx: a .trk.bad converts; damaged bytes and bad times dropped",
      rc == 0 and gpx_out.count("<trkpt") == 1
      and "<name>20261029_101500</name>" in gpx_out, gpx_out[:300])
check("trk2gpx: a non-digit time is not a time",
      trk2gpx._iso("2026102XT101501Z") is None
      and trk2gpx._iso("20261029T101501Z") == "2026-10-29T10:15:01Z")
shutil.rmtree(t_td, ignore_errors=True)


# 2026-10-30: three WDT resets during uploads; GPS restart storms
class RvDog:
    def __init__(self):
        self.n = 0

    def feed(self):
        self.n += 1


saved_wdt = gl._wdt
dog = gl._wdt = RvDog()
try:
    # POST-1: a write that keeps saying "would block" times out, fed
    class RvNone:
        def __init__(self):
            self.calls = 0

        def write(self, b):
            self.calls += 1
            return None
    rn = RvNone()
    t0 = _clock[0]
    try:
        gl._io_write(rn, b"x" * 100)
        res = None
    except OSError as e:
        res = e.args[0]
    check("POST-1: a write stuck at 'would block' ends with ETIMEDOUT",
          res == 110 and _clock[0] - t0 >= gl.HTTP_TIMEOUT * 1000
          and dog.n > 10 and rn.calls < 5000, "%r %d %d" % (res, dog.n,
                                                             rn.calls))

    # POST-2: the guard feeds the dog until the deadline, then closes
    class RvClose:
        closed = 0

        def close(self):
            RvClose.closed += 1
    gl._guard["t"] = None
    sock = RvClose()
    gl._guard_start(sock, gl.HTTP_MAX)
    timer_ok = gl._guard["t"] is not None
    n0, h0 = dog.n, gl._guard["hits"]
    for _ in range(gl.HTTP_MAX // 1000 - 1):
        _clock[0] += 1000
        gl._guard_cb(None)
    fed_meanwhile = dog.n - n0
    closed_early = RvClose.closed
    _clock[0] += 2000
    gl._guard_cb(None)
    check("POST-2: the guard feeds the WDT, then closes a POST over HTTP_MAX",
          timer_ok and fed_meanwhile >= gl.HTTP_MAX // 1000 - 2
          and closed_early == 0 and RvClose.closed == 1
          and gl._guard["hits"] == h0 + 1 and gl._guard["until"] is None,
          "%d %d %d" % (fed_meanwhile, closed_early, RvClose.closed))

    # POST-3: _raw_post arms the guard and always disarms it
    rv_sock[0] = RvSock(b"", polls=0)
    gl.socket, gl.select = fake_sockmod, fake_sel
    sys.modules["tls"] = RvTls
    armed = []
    real_gs = gl._guard_start
    gl._guard_start = lambda s_, ms: (armed.append(ms), real_gs(s_, ms))
    try:
        try:
            gl._raw_post("https://d.example", b"{}", {}, 15)
        except Exception:
            pass
    finally:
        gl._guard_start = real_gs
        gl.socket, gl.select = real_mods[0], real_mods[1]
        sys.modules["tls"] = real_mods[2]
        tls_ctx_reset()
    check("POST-3: each POST is guarded and the guard is off afterwards",
          armed == [gl.HTTP_MAX] and gl._guard["until"] is None, repr(armed))
finally:
    gl._wdt = saved_wdt

# POST-4: an upload error goes to the diag log once per kind
diag_clear()
gl.DIAG = 60000
online(up_td)
gl.up["url"], gl.up["key"] = "https://d.example", "k"
for _ in range(3):
    _post_raise[0] = OSError(116)
    gl._http_post(10)
_post_raise[0] = OSError(113)
gl._http_post(10)
gl._http_post(10)                        # a success clears it
_post_raise[0] = OSError(113)
gl._http_post(10)
errs = [l for l in diag_lines() if l.startswith("upload err")]
check("POST-4: upload errors in the diag log, one line per kind",
      errs == ["upload err OSError(116)", "upload err OSError(113)",
               "upload err OSError(113)"], repr(errs))
_post_raise[0] = None

# POST-5: the guard closing the socket while the headers are read keeps the
# 2xx; the next, unrelated error must not report "aborted after 60 s"
diag_clear()
real_hp = gl.HTTP_POST


def _post_guarded(url, data, headers, timeout):
    gl._guard["hits"] += 1               # fired in the header loop
    return FakeResp(201)


gl.HTTP_POST = _post_guarded
try:
    g_code = gl._http_post(10)[0]
finally:
    gl.HTTP_POST = real_hp
_post_raise[0] = OSError(-202)           # a DNS failure, seconds later
gl._http_post(10)
_post_raise[0] = None
g_lines = [l for l in diag_lines() if l.startswith("upload")]
check("POST-5: a guard hit on a 2xx is no 'aborted' line for the next error",
      g_code == 201 and g_lines == ["upload err OSError(-202)"],
      repr(g_lines))

# GPS-STORM: while it loops, setups are GPS_STORM_WAIT apart; diag lines
diag_clear()
gl.reset()
banner = rv_nmea("GPTXT,01,01,02,u-blox ag - www.u-blox.com")
gl.handle(banner)
calm = gl.st["setup_wait"]
for _ in range(gl.GPS_FLAP_RESTARTS):
    _clock[0] += 5000
    gl.handle(banner)
gl.handle(banner)
storm = gl.st["setup_wait"]
lines = diag_lines()
check("GPS-STORM: setups GPS_STORM_WAIT apart while the receiver loops",
      calm == gl.GPS_RETRY and storm == gl.GPS_STORM_WAIT
      and any(l.startswith("gps reset loop ") for l in lines),
      "%r %r %r" % (calm, storm, lines[-2:]))
gl.DIAG = 0
diag_clear()


# LOWIDF: no TLS try below TLS_MIN_KB; at home gc.collect() and wait first
online(up_td)
gl.up["url"], gl.up["key"] = "https://d.example", "k"
make_trk(os.path.join(up_td, "20261007_100000.trk"), 3)
real_idf = gl.idf_heap
gl.idf_heap = lambda: (30, 20)
_posts[:] = []
_resets[:] = []
try:
    gl.wifi["home"] = False
    pump()
    away = (len(_posts), gl.up["state"], len(_resets))
    gl.wifi["home"] = True
    try:
        pump()
    except SystemExit:
        pass
    home = (len(_posts), len(_resets), gl.up["state"],
            gl.up["next_at"] - _clock[0])
finally:
    gl.idf_heap = real_idf
    gl.wifi["home"] = False
check("LOWIDF: below TLS_MIN_KB no POST; away it waits, at home too at first",
      away == (0, "low idf", 0)
      and home == (0, 0, "low idf", getattr(gl, "LOWIDF_RETRY", -1)),
      "%r %r" % (away, home))

# LOWIDF-WAIT: at home the reboot comes only after LOWIDF_WAIT of low idf
# with tracks waiting, and it tells the next boot (LOWIDF_MARK)
wait_ms = getattr(gl, "LOWIDF_WAIT", 300000)
mark = getattr(gl, "LOWIDF_MARK", os.path.join(diag_dir, "lowidf"))
if os.path.exists(mark):
    os.remove(mark)
online(up_td)
gl.up["url"], gl.up["key"] = "https://d.example", "k"
make_trk(os.path.join(up_td, "20261007_100000.trk"), 3)
gl.idf_heap = lambda: (30, 20)
gl.wifi["home"] = True
_posts[:] = []
_resets[:] = []
steps = []
try:
    for dt in (0, wait_ms // 2, wait_ms // 2 - 1000, 2000):
        _clock[0] += dt
        try:
            pump()
        except SystemExit:
            pass
        steps.append(len(_resets))
finally:
    gl.idf_heap = real_idf
    gl.wifi["home"] = False
check("LOWIDF-WAIT: at home a reboot only after LOWIDF_WAIT of low idf",
      steps == [0, 0, 0, 1] and not _posts and os.path.exists(mark),
      "%r posts %d mark %s" % (steps, len(_posts), os.path.exists(mark)))

# LOWIDF-RATE: after a low-idf reboot the next one waits LOWIDF_REBOOT_GAP
online(up_td)                            # up_reset at boot takes the mark
gl.up["url"], gl.up["key"] = "https://d.example", "k"
make_trk(os.path.join(up_td, "20261007_100000.trk"), 3)
gap_ms = getattr(gl, "LOWIDF_REBOOT_GAP", 1800000)
took = (gl.up.get("lowidf_boot"), os.path.exists(mark))
saved_up = list(gl._up)
gl._up[0], gl._up[1] = 0, _clock[0]      # just booted
gl.idf_heap = lambda: (30, 20)
gl.wifi["home"] = True
_resets[:] = []
t_boot = _clock[0]
t_reboot = None
retry_ms = getattr(gl, "LOWIDF_RETRY", 30000)
try:
    while _clock[0] - t_boot < gap_ms + 4 * retry_ms:
        try:
            pump()                       # low idf the whole time
        except SystemExit:
            t_reboot = _clock[0] - t_boot
            break
        _clock[0] += retry_ms
finally:
    gl.idf_heap = real_idf
    gl.wifi["home"] = False
    gl._up[:] = saved_up
check("LOWIDF-RATE: at most one low-idf reboot per LOWIDF_REBOOT_GAP",
      took == (True, False) and t_reboot is not None
      and gap_ms <= t_reboot <= gap_ms + retry_ms,
      "%r %r" % (took, t_reboot))
if os.path.exists(mark):
    os.remove(mark)

# LOWIDF-BACK: the IDF heap recovering ends the episode; a new one starts
# its LOWIDF_WAIT again
online(up_td)
gl.up["url"], gl.up["key"] = "https://d.example", "k"
make_trk(os.path.join(up_td, "20261007_100000.trk"), gl.UPLOAD_BATCH * 3)
low_now = [True]
gl.idf_heap = lambda: (30, 20) if low_now[0] else (157, 126)
gl.wifi["home"] = True
_resets[:] = []
_posts[:] = []
try:
    for dt, low_ in ((0, True),          # low: episode starts
                     (wait_ms - 1000, False),    # back: a POST goes
                     (2000, True),       # low again: a new episode
                     (wait_ms - 1000, True)):
        _clock[0] += dt
        low_now[0] = low_
        try:
            pump()
        except SystemExit:
            pass
finally:
    gl.idf_heap = real_idf
    gl.wifi["home"] = False
check("LOWIDF-BACK: a recovered IDF heap restarts the LOWIDF_WAIT",
      not _resets and len(_posts) == 1, "resets %d posts %d" % (
          len(_resets), len(_posts)))

# LOWIDF-GC: a grown, now empty Python heap area goes back on gc.collect()
# (MicroPython 1.29 frees empty split-heap areas in the sweep): check again
# after a collect before giving up on TLS
online(up_td)
gl.up["url"], gl.up["key"] = "https://d.example", "k"
make_trk(os.path.join(up_td, "20261007_110000.trk"), 3)
collected = [0]
real_collect = gl.gc.collect


def _collect():
    collected[0] += 1
    return real_collect()


gl.idf_heap = lambda: (157, 126) if collected[0] else (30, 20)
gl.gc.collect = _collect
_posts[:] = []
_resets[:] = []
try:
    gl.wifi["home"] = True
    pump()
finally:
    gl.idf_heap = real_idf
    gl.gc.collect = real_collect
    gl.wifi["home"] = False
check("LOWIDF-GC: low IDF heap -> gc.collect() first, then the upload goes",
      len(_posts) >= 1 and not _resets and gl.up["state"] != "low idf",
      "%d posts %d resets %r" % (len(_posts), len(_resets), gl.up["state"]))

# LOWPY: under TLS_MIN_PY_KB of free Python heap after the collect the POST
# waits ("low mem"): its allocations would grow the heap into the IDF block
online(up_td)
gl.up["url"], gl.up["key"] = "https://d.example", "k"
lp_p = os.path.join(up_td, "20261007_120000.trk")
make_trk(lp_p, 3)
real_idf_info = getattr(gl, "_idf_info", None)
real_mem_free = gl.gc.mem_free
gl._idf_info = lambda: (157 * 1024, 126 * 1024)
gl.idf_heap = lambda: (157, 126)
gl.gc.mem_free = lambda: 126 * 1024 + 8000
_posts[:] = []
diag_clear()
gl.DIAG = 60000
try:
    pump()
    low = (len(_posts), gl.up["state"], os.path.exists(lp_p))
    lp_waits = [gl.up["next_at"] - _clock[0]]
    for _ in range(2):
        pump()
        lp_waits.append(gl.up["next_at"] - _clock[0])
    gl.gc.mem_free = lambda: 126 * 1024 + 20000
    pump()
    ok_ = (len(_posts), os.path.exists(lp_p))
finally:
    gl.gc.mem_free = real_mem_free
    gl.idf_heap = real_idf
    if real_idf_info is None:
        del gl._idf_info
    else:
        gl._idf_info = real_idf_info
    gl.DIAG = 0
lp_diag = [l for l in diag_lines() if l.startswith("upload low mem")]
diag_clear()
check("LOWPY: below TLS_MIN_PY_KB free Python heap the POST waits, then goes",
      low == (0, "low mem", True) and ok_ == (1, False)
      and lp_diag == ["upload low mem 8000 B"], "%r %r %r" % (low, ok_,
                                                               lp_diag))
check("LOWPY-RETRY: low mem retries after LOWIDF_RETRY each time (no "
      "doubling backoff)", lp_waits == [gl.LOWIDF_RETRY] * 3, repr(lp_waits))
# LOWPY-FROZEN: with the logger frozen in (gps_frozen) the POST needs less
import importlib.util
sys.modules["gps_frozen"] = types.ModuleType("gps_frozen")
try:
    spec_ = importlib.util.spec_from_file_location(
        "gps_logger_frozen_probe", os.path.join(ROOT, "gps_logger.py"))
    probe = importlib.util.module_from_spec(spec_)
    spec_.loader.exec_module(probe)
    fz_ = (getattr(probe, "FROZEN", None), probe.TLS_MIN_PY_KB)
finally:
    del sys.modules["gps_frozen"]
check("LOWPY-FROZEN: TLS_MIN_PY_KB is 8 on frozen firmware, 12 otherwise",
      fz_ == (True, 8) and (getattr(gl, "FROZEN", None), gl.TLS_MIN_PY_KB)
      == (False, 12), "%r %r" % (fz_, gl.TLS_MIN_PY_KB))

# HEAP: gc.mem_free() includes the largest free IDF block (modgc.c), so the
# diag showed a flat "mem" while the heap doubled into the IDF heap.
# HEAP-B: to the byte, from esp32.idf_heap_info (not the KB-floored block)
real_idf_info = getattr(gl, "_idf_info", None)
gl._idf_info = lambda: (157 * 1024 + 300, 102400 + 700)
gl.idf_heap = lambda: (157, 100)
lines = []
real_dw = gl._diag_write
gl._diag_write = lines.append
try:
    ph = gl.py_heap()
    gl._diag(_clock[0])
    st_mem = dict(gl._web_rows())["mem"]
finally:
    gl.idf_heap = real_idf
    if real_idf_info is None:
        del gl._idf_info
    else:
        gl._idf_info = real_idf_info
    gl._diag_write = real_dw
check("HEAP: free and size of the Python heap without the IDF block",
      ph[1] == (65432 + 123456 - 103100) // 1024
      and " mem %d " % ph[0] in lines[0] and lines[0].endswith(" heap 83")
      and " idf 157/100 " in lines[0] and st_mem == ph[0],
      "%r %r" % (ph, lines))
lines = []
real_dw = gl._diag_write
gl._diag_write = lines.append
saved_pvt = (gl.st["mode"], gl.st["hacc"], gl.st["lat_i"], gl.st["ubx_ok"],
             gl.st["ubx_bad"])
try:
    gl.st.update(mode="pvt", hacc=2.34, lat_i=471000000, ubx_ok=646,
                 ubx_bad=1)
    gl._diag(_clock[0])
    gl.st["lat_i"] = None                # no fix this second
    gl._diag(_clock[0])
    gl.st["mode"] = "nmea"
    gl._diag(_clock[0])
finally:
    (gl.st["mode"], gl.st["hacc"], gl.st["lat_i"], gl.st["ubx_ok"],
     gl.st["ubx_bad"]) = saved_pvt
    gl._diag_write = real_dw
check("DIAG-PVT: on NAV-PVT the minute line adds ubx good/bad and hAcc",
      lines[0].endswith(" ubx 646/1 hacc 2.3")
      and lines[1].endswith(" ubx 646/1 hacc -")
      and " ubx " not in lines[2] and " hacc " not in lines[2],
      repr(lines))
check("HEAP-B: the IDF block comes off to the byte, not KB-floored",
      ph[0] == 123456 - 103100, "%r" % (ph,))

# HEAP-LINE: the upload console line shows the Python heap's own free bytes
# HEAP-GREW: a diag line names the upload step that grew the Python heap
online(up_td)
gl.up["url"], gl.up["key"] = "https://d.example", "k"
hg_p = os.path.join(up_td, "20261008_100000.trk")
make_trk(hg_p, 3)
heap_kb = [124]
real_mem_free, real_mem_alloc = gl.gc.mem_free, gl.gc.mem_alloc
gl.gc.mem_free = lambda: 30000 + 102900
gl.gc.mem_alloc = lambda: heap_kb[0] * 1024 - 30000
gl._idf_info = lambda: (157 * 1024, 102900)
real_rb_ = gl.read_batch


def _rb_grows(path, off=0):
    heap_kb[0] = 248                     # the read grew the heap
    return real_rb_(path, off)


gl.read_batch = _rb_grows
printed = []
real_print = (print if not isinstance(__builtins__, dict)
              else __builtins__["print"])


def _cap_print(*a, **k):
    printed.append(" ".join(str(x) for x in a))


diag_clear()
gl.DIAG = 60000
if isinstance(__builtins__, dict):
    __builtins__["print"] = _cap_print
else:
    __builtins__.print = _cap_print
try:
    pump()
finally:
    if isinstance(__builtins__, dict):
        __builtins__["print"] = real_print
    else:
        __builtins__.print = real_print
    gl.read_batch = real_rb_
up_lines = [l for l in printed if l.startswith("upload " + hg_p)]
check("HEAP-LINE: the upload line prints py_heap()[0], not gc.mem_free()",
      len(up_lines) == 1 and up_lines[0].split()[5] == "30000", repr(up_lines))
grew = [l for l in diag_lines() if l.startswith("heap grew")]
diag_clear()
# ... and in the TLS handshake of the real POST
online(up_td)
gl.up["url"], gl.up["key"] = "https://d.example", "k"


class RvTlsGrow(RvTls):
    class SSLContext(RvTls.SSLContext):
        def wrap_socket(self, s, server_hostname=None):
            heap_kb[0] = 300
            return s


heap_kb[0] = 248
gl.socket, gl.select = fake_sockmod, fake_sel
saved_tls = sys.modules.get("tls")
sys.modules["tls"] = RvTlsGrow
tls_ctx_reset()
real_hp = gl.HTTP_POST
gl.HTTP_POST = gl._raw_post
try:
    rv_sock[0] = RvSock(b"HTTP/1.1 201 Created\r\n\r\n", polls=0)
    make_trk(hg_p, 3)
    pump()
finally:
    gl.HTTP_POST = real_hp
    gl.socket, gl.select = real_mods[0], real_mods[1]
    if saved_tls is None:
        sys.modules.pop("tls", None)
    else:
        sys.modules["tls"] = saved_tls
    tls_ctx_reset()
    gl.gc.mem_free, gl.gc.mem_alloc = real_mem_free, real_mem_alloc
    if real_idf_info is None:
        del gl._idf_info
    else:
        gl._idf_info = real_idf_info
    gl.DIAG = 0
grew = grew + [l for l in diag_lines() if l.startswith("heap grew")]
diag_clear()
check("HEAP-GREW: diag names the step that grew the heap (read, TLS)",
      grew == ["heap grew in read_batch to 248 KB",
               "heap grew in tls_wrap to 300 KB"]
      and not os.path.exists(hg_p), repr(grew))

# HEAP-LABELS: the connect phase and the whole POST have labels of their own
# (both were "http_post")
online(up_td)
gl.up["url"], gl.up["key"] = "https://d.example", "k"


class RvSockGrow(RvSock):
    def connect(self, addr):
        heap_kb[0] = 260                 # the TCP connect grew it
        RvSock.connect(self, addr)

    def readline(self, n=-1):
        heap_kb[0] = 280                 # the answer grew it
        return RvSock.readline(self, n)


heap_kb[0] = 248
gl.gc.mem_free = lambda: 30000 + 102900
gl.gc.mem_alloc = lambda: heap_kb[0] * 1024 - 30000
gl._idf_info = lambda: (157 * 1024, 102900)
gl.socket, gl.select = fake_sockmod, fake_sel
real_hp = gl.HTTP_POST
gl.HTTP_POST = gl._raw_post
diag_clear()
gl.DIAG = 60000
try:
    rv_sock[0] = RvSockGrow(b"HTTP/1.1 201 Created\r\n\r\n", polls=0)
    make_trk(hg_p, 3)
    pump()
finally:
    gl.HTTP_POST = real_hp
    gl.socket, gl.select = real_mods[0], real_mods[1]
    gl.gc.mem_free, gl.gc.mem_alloc = real_mem_free, real_mem_alloc
    if real_idf_info is None:
        del gl._idf_info
    else:
        gl._idf_info = real_idf_info
    gl.DIAG = 0
grew = [l for l in diag_lines() if l.startswith("heap grew")]
diag_clear()
check("HEAP-LABELS: diag says 'connect' and 'post', not 'http_post' twice",
      grew == ["heap grew in connect to 260 KB",
               "heap grew in post to 280 KB"], repr(grew))


# GPS-STORM-END: quiet for GPS_STORM_QUIET after a storm -> set up soon
gl.reset()
banner = rv_nmea("GPTXT,01,01,02,u-blox ag - www.u-blox.com")
for _ in range(gl.GPS_FLAP_RESTARTS + 1):
    _clock[0] += 3000
    gl.handle(banner)
gl.st["setup_at"] = _clock[0]
storm_wait = gl.st["setup_wait"]
_uart_tx[:] = []
_clock[0] += gl.GPS_STORM_QUIET + gl.GPS_RETRY + 500
gl.gps_check(_clock[0])
sent = len([f for f in _uart_tx if f[2:3] == b"\x06"]) > 0
check("GPS-STORM-END: after the storm the setup comes within ~15 s, not 30-60",
      storm_wait == gl.GPS_STORM_WAIT and sent, "%r %r" % (storm_wait, sent))

# STORM-END-NOACK: the last storm setup got no ACK, but the receiver took
# the blind burst (no extra sentence): quiet after the storm, it is still
# set up within GPS_RETRY, not after the doubled GPS_STORM_WAIT
gl.reset()
for _ in range(gl.GPS_FLAP_RESTARTS + 1):
    _clock[0] += 3000
    gl.handle(banner)
_uart_ack[0] = False
try:
    gl.gps_setup()
finally:
    _uart_ack[0] = True
noack = (gl.st["setup_ok"], gl.st["extra"], gl.st["setup_wait"])
_uart_tx[:] = []
_clock[0] += gl.GPS_STORM_QUIET + gl.GPS_RETRY + 500
gl.gps_check(_clock[0])
sent = len([f for f in _uart_tx if f[2:3] == b"\x06"]) > 0
check("STORM-END-NOACK: unacknowledged last storm setup, quiet -> set up soon",
      noack == (False, 0, gl.GPS_STORM_WAIT) and sent, "%r %r" % (noack, sent))

# STORM-FIRST: the restart that makes it a storm already gets the storm wait
gl.reset()
for _ in range(gl.GPS_FLAP_RESTARTS):
    _clock[0] += 5000
    gl.handle(banner)
check("STORM-FIRST: the restart that declares the storm waits GPS_STORM_WAIT",
      gl.st["loops"] == 1 and gl.st["setup_wait"] == gl.GPS_STORM_WAIT,
      "loops=%d wait=%d" % (gl.st["loops"], gl.st["setup_wait"]))

# STORM-SPACED: restarts further apart than GPS_STORM_QUIET (12 s) each got
# a setup of their own: 50 restarts, 50 setups in 10 min, a third of the
# loop blocked. In a storm quiet is twice the last gap (up to
# GPS_STORM_WAIT): setups GPS_STORM_WAIT apart; restarts further apart
# than that are still each set up


def storm_run(gap):
    gl.reset()
    calls = []
    real = gl.gps_setup

    def fake():
        calls.append(_clock[0])
        gl.st.update(setup_at=_clock[0], extra=0, setup_ok=True, checked=True)

    gl.gps_setup = fake
    try:
        gl.st.update(setup_at=_clock[0], setup_ok=True, extra=0, checked=True)
        end = _clock[0] + 600000
        nxt = _clock[0]
        k = 0
        while _clock[0] < end:
            if _clock[0] >= nxt:
                gl.handle(banner)
                nxt += gap
            k += 1
            if k % 5 == 0:
                gl.handle(rv_nmea("GPRMC,120000.00,V,,,,,,,311026,,,N"))
            gl.gps_check(_clock[0])
            _clock[0] += 200
    finally:
        gl.gps_setup = real
    return len(calls)


s12, s40 = storm_run(12000), storm_run(40000)
check("STORM-SPACED: restarts 12 s apart: setups GPS_STORM_WAIT apart",
      s12 <= 600000 // gl.GPS_STORM_WAIT + 3 and s40 == 15,
      "12 s: %d setups for 50 restarts, 40 s: %d for 15" % (s12, s40))

# WRAP-GPS: ticks_ms wraps every 2**30 ms; the storm mark, old restart
# times and a satellite list must not read as fresh again 12.4 days later
TPW = 1 << 30
saved_td = time.ticks_diff
time.ticks_diff = lambda a, b: ((a - b + TPW // 2) & (TPW - 1)) - TPW // 2
try:
    gl.reset()
    for _ in range(2 * gl.GPS_FLAP_RESTARTS - 1):   # a storm + 4 restarts
        _clock[0] += 3000
        gl.handle(banner)
    gl.st.update(svs=[(k, 30, 40, 100, True, False, 0) for k in range(9)],
                 svs_at=_clock[0], used=7)
    base = _clock[0]
    loops0 = gl.st["loops"]
    for dt in (gl.GPS_FLAP_WINDOW + 10000,
               getattr(gl, "TICKS_CAP", 86400000) + 10000):
        _clock[0] = base + dt
        gl.gps_check(_clock[0])
    _clock[0] = base + TPW + 5000            # all would read 5-30 s old
    used = gl.sv_used()
    gl.handle(banner)
    wrapped = (gl.st["setup_wait"], gl.st["loops"] - loops0, used)
finally:
    time.ticks_diff = saved_td
check("WRAP-GPS: storm mark, restart times, satellite list end, not wrap",
      wrapped == (gl.GPS_RETRY, 0, 7), repr(wrapped))

# LOOP-DIAG: a banner-only reset loop wrote a diag line every
# GPS_LOOP_WINDOW (12.6 KB an hour): now the first warning, a count every
# GPS_LOOP_SAY and one when it ends; every window is still counted
diag_clear()
gl.DIAG = 60000
gl.reset()
_uart_rx[:] = []
for _ in range(40):
    for _ in range(3):
        _uart_rx.extend(b"$GPTXT,01,01,02,u\xff\xff\xff")
        gl.read_uart()
    _clock[0] += gl.GPS_LOOP_WINDOW
    gl.gps_check(_clock[0])
for k in range(3):
    _uart_rx.extend(rv_nmea("GPRMC,1208%02d.00,V,,,,,,,171026,,,N" % k)
                    + b"\r\n")
gl.read_uart()
_clock[0] += gl.GPS_LOOP_WINDOW
gl.gps_check(_clock[0])
loop_lines = [l for l in diag_lines() if l.startswith("gps reset loop")]
check("LOOP-DIAG: first warning, then one line per GPS_LOOP_SAY and at the end",
      gl.st["loops"] == 40 and loop_lines
      == ["gps reset loop 3 banners, no NMEA",
          "gps reset loop 30 more windows", "gps reset loop 9 more windows"],
      "loops=%d %d lines %r" % (gl.st["loops"], len(loop_lines),
                                loop_lines[:3]))
gl.DIAG = 0
diag_clear()


# PAGE: web.html draws the boxes from /status.json (node runs its view())
gl.reset()
gl.up_reset()
gl.wifi_reset()
gl.st.update(rx=_clock[0], valid=True, fixtype=3, quality=1,
             hdop=0.88199996, lat=47.1, lon=18.4)
gl.st["svs"] = [(16, 26, 0, 90, True, False, 0),
                (11, 12, 90, 250, False, False, 3),
                (5, 0, None, 0, False, False, 1)]
gl.st["svs_at"] = _clock[0]
gl.up["left"] = 120
gl.wifi.update(state="connected", ssid="<x>", rssi=-75)
_diag_saved = gl._web_diag
gl._web_diag = lambda: ["up 60 utc 20261031T053014Z nmea 9/1 gpsboots 0 "
                         "loops 0 sats 21/13 fix 3D_fix hdop 0.9 points 4 "
                         "sent 0 wifi connected pulses 0 mem 27000 idf "
                         "157/20 heap 124", "boot cause 3 prev_up 600 mhz 80",
                         "upload ca err OSError(5)", "up 60 <b>"]
_idf_saved = gl.idf_heap
gl.idf_heap = lambda: (157, 144)
page_json = gl._web_json()
gl.idf_heap = _idf_saved
gl.st["valid"] = False
gl.wifi.update(state="off", ssid=None, rssi=None)
gl.st["svs"] = None
page_json2 = gl._web_json()
# a NAV-PVT second (new feature): hAcc, the fix source, UBX frame counts
gl.st.update(valid=True, fixtype=3, quality=1, hdop=None, lat=47.1,
             lon=18.4, lat_i=471000000, lon_i=184000000, hacc=30.04,
             fix_src="pvt", ubx_ok=500, ubx_bad=2)
gl._web_diag = lambda: ["up 120 utc 20261031T130241Z nmea 72/0 gpsboots 0 "
                         "loops 0 sats 22/15 fix 3D_fix hdop None points 0 "
                         "sent 0 wifi connected pulses 0 mem 19280 idf "
                         "221/208 heap 62 ubx 646/0 hacc 2.3"]
page_json3 = gl._web_json()
gl._web_diag = _diag_saved
jd1, jd3 = json.loads(page_json), json.loads(page_json3)
check("PAGE: JSON is generated piece by piece (no list in RAM)",
      not isinstance(gl._web_json_parts(), (list, tuple))
      and jd1["hdop"] == 0.88199996)
check("PAGE-PVT: status JSON hacc (m), fix_src and ubx ok / bad; nmea kept",
      jd3["hacc"] == 30.0 and jd3["fix_src"] == "pvt"
      and jd3["ubx"] == "500 / 2" and jd3["hdop"] is None
      and "nmea" in jd3 and jd1["hacc"] is None and jd1["ubx"] == "0 / 0",
      repr((jd3.get("hacc"), jd3.get("fix_src"), jd3.get("ubx"),
            jd1.get("hacc"))))
gl.reset()
import shutil
import subprocess
node = shutil.which("node")
if node is None and os.environ.get("CI") == "true":
    check("PAGE: node is installed (CI runs the page checks, never skips)",
          False)
elif node is None:
    print("SKIP  PAGE: node not installed")
else:
    here = os.path.dirname(os.path.abspath(__file__))
    out = []
    for js in (page_json, page_json2, page_json3):
        r = subprocess.run([node, os.path.join(here, "web_page_test.js"),
                            os.path.join(here, "..", "web.html")],
                           input=js, capture_output=True, text=True)
        try:
            out.append(json.loads(r.stdout))
        except ValueError:
            out.append({"error": r.stdout + r.stderr})
    v, v2, v3 = out
    tm, tm2 = v.get("tm", ""), v2.get("tm", "")
    tm3, sy3 = v3.get("tm", ""), v3.get("sys", "")
    check("PAGE-PVT: telemetry shows +-hAcc m (amber over PVT_MAX_HACC), "
          "HDOP without it",
          '<div class="warn"><small>ACC</small><b>&plusmn;30.0<em>m</em>'
          '</b></div>' in tm3 and "<small>HDOP</small>" not in tm3
          and "<small>ACC</small>" not in tm and "<small>HDOP</small>" in tm,
          tm3[:400])
    check("PAGE-PVT: map ring from hAcc as it is, coordinates +-hAcc m",
          v3.get("dom", {}).get("acc") == "74px"
          and v3.get("coord", "").endswith("&plusmn;30.0 m")
          and v.get("coord", "").endswith("&plusmn;2.2 m"),
          repr((v3.get("dom", {}).get("acc"), v3.get("coord"),
                v.get("coord"))))
    check("PAGE-PVT: on NAV-PVT the strip shows UBX BAD, not the frozen "
          "NMEA count; the log row shows +-hAcc",
          "<small>UBX BAD</small><b>0.40<em>%</em>" in tm3
          and "NMEA BAD" not in tm3 and "<small>NMEA BAD</small>" in tm
          and "15/22 SATS &middot; &plusmn;2.3 M &middot; IDF 208 KB"
          in v3.get("log", ""), tm3[-300:] + v3.get("log", "")[:300])
    check("PAGE-PVT: Systems shows the fix source and the UBX frame counts",
          '>FIX SOURCE<span class="">NAV-PVT<em>ubx 500 ok &middot; 2 bad'
          '</em>' in sy3 and v3.get("unlabeled") == []
          and '>FIX SOURCE<span class="">-<em>ubx 0 ok' in v.get("sys", ""),
          sy3[-900:])
    check("PAGE: telemetry strip - fix, sats, HDOP, uplink, WiFi, NMEA bad",
          '<div class="ok"><small>FIX</small><b>3D</b></div>' in tm
          and "<small>SATS</small><b>1<em>/2</em>" in tm
          and "<small>HDOP</small><b>0.88</b>" in tm
          and "<small>UPLINK</small><b>120<em>pts</em>" in tm
          and '<div class="warn"><small>WIFI</small><b><svg class="wi" viewBox="0 0 24 18">'
              '<circle cx="12" cy="15.5" r="2"/><path class="on" d="M8.5 12.5A5 5 0 0 1 15.5 12.5"/>'
              '<path d="M5.6 9.6A9 9 0 0 1 18.4 9.6"/><path d="M2.8 6.8A13 13 0 0 1 21.2 6.8"/>'
              '</svg>50<em>%</em><em class="db">-75 dBm</em></b></div>' in tm
          and "<small>NMEA BAD</small>" in tm, tm[:400])
    check("PAGE: no fix amber, WiFi off shows the state, map says NO FIX",
          '<div class="warn"><small>FIX</small><b>WAITING</b>' in tm2
          and "<small>WIFI</small><b>OFF</b>" in tm2
          and v2.get("coord", "").startswith("NO FIX")
          and v.get("coord", "").startswith("47.10000&deg; N  18.40000&deg; E")
          and "NO SATELLITE LIST" in v2.get("bars", ""), tm2[:400])
    check("PAGE: sky radar - horizon east at x 90, zenith centre, hollow unused",
          '<circle class="g0" cx="90" cy="0" r="3.4"><title>GPS 16 26 dBHz'
          in v.get("dots", "")
          and '<circle class="g3 u" cx="0" cy="0" r="2.4">' in v.get("dots", "")
          and "SBAS 5" not in v.get("dots", "-")
          and v.get("skyh") == "Sky &middot; 1 locked / 3 tracked",
          v.get("dots", "")[:300])
    check("PAGE: one bar per satellite (45 dBHz full, stub for none)",
          '<i class="g0" style="height:57%" title="GPS 16 26 dBHz">'
          in v.get("bars", "")
          and '<i class="g3 u" style="height:26%"' in v.get("bars", "")
          and '<i class="g1 u" style="height:0%" title="SBAS 5 - dBHz">'
          in v.get("bars", ""), v.get("bars", "")[:300])
    sy = v.get("sys", "")
    check("PAGE: Systems - every JSON key shown, escaped, IDF bar with mark",
          v.get("unlabeled") == [] and ">IDF BLOCK<" in sy
          and '<b style="left:30%">' in sy and "&lt;x&gt;" in sy
          and "<x>" not in sy and ">UPLINK<" in sy and "IDLE" in sy
          and ">RECEIVER<" in sy and ">BOOT<" in sy,
          repr(v.get("unlabeled")) + sy[:300])
    lg = v.get("log", "")
    check("PAGE: Event log - newest first, status rows, events, red low IDF",
          lg.startswith('<div><span class="t">--:--</span>  <span class="">'
                        'UP 60 &lt;B&gt;')
          and lg.index("BOOT &middot; WATCHDOG &middot; PREVIOUS RUN 10 MIN 00 S")
              < lg.index("3D FIX &middot; 13/21 SATS &middot; 4 PTS")
          and '<span class="err">IDF 20 KB</span>' in lg, lg[:500])
    check("PAGE: Event log - an unreadable CA_FILE ('upload ca err') is red",
          '<span class="err">UPLOAD CA ERR OSERROR(5)</span>' in lg, lg[:600])
    # PAGE-WIFI: an arc icon (0-3 arcs lit) and the quality in per cent,
    # 2 x (RSSI + 100) clamped; amber below -67 dBm, red below -80; no words
    check("PAGE-WIFI: arcs, quality % and tone from RSSI",
          v.get("wq") == [[3, 100, ""], [3, 80, ""], [2, 74, ""], [2, 66, ""],
                          [1, 64, "warn"], [1, 40, "warn"], [0, 38, "bad"],
                          [0, 0, "bad"], [0, 0, "bad"], None],
          repr(v.get("wq")))
    check("PAGE-WIFI: Systems line 'ssid · 50% (-75 dBm)', no quality words",
          "&lt;x&gt; &middot; 50% (-75 dBm)<em>" in sy
          and not any(w in tm + sy for w in ("GOOD", "FAIR", "WEAK",
                                             "EXCELLENT", "NO SIGNAL")),
          sy[:600])
    # PAGE-NULL: south/west coordinates, Python None in diag lines, quotes
    check("PAGE-NULL: S/W coordinates as absolute values",
          v.get("coordSW", "").startswith("33.85678&deg; S  151.21530&deg; W")
          and v.get("coordNE", "") == "47.10000&deg; N  18.40000&deg; E",
          repr((v.get("coordSW"), v.get("coordNE"))))
    check("PAGE-NULL: diag 'sats 0/None' reads -/0, 'idf None' and 'cause "
          "None' read -",
          "-/0 SATS" in v.get("logNone", "") and "IDF - KB" in v.get("logNone", "")
          and "None" not in v.get("logNone", "") + v.get("bootNone", "")
          and "CAUSE -" in v.get("bootNone", ""),
          repr((v.get("logNone"), v.get("bootNone"))))
    check("PAGE-NULL: an empty status.json shows - (no None/null/NaN)",
          'PY HEAP FREE<span class="">- / - KB</span>' in v.get("nullSys", "")
          and 'WIFI<span class="">-<em>' in v.get("nullSys", "")
          and not any(w in v.get("nullSys", "") for w in (
              "None", "null", "NaN", "undefined")),
          v.get("nullSys", "")[:600])
    check("PAGE-NULL: esc() also escapes \" and '",
          v.get("esc") == "&lt;a href=&quot;x&quot; title=&#39;y&#39;&gt;"
                          "&amp;&lt;/a&gt;", repr(v.get("esc")))
    dom = v.get("dom", {})
    check("PAGE-DOM: tick/paint/placeMap/stale on a stub document",
          dom.get("first") and dom.get("painted") and dom.get("mapClass") == "map"
          and dom.get("tilesWide") == 55 and dom.get("tilesNarrow") == 25
          and dom.get("kept") and dom.get("live") == "LIVE"
          and dom.get("body") == "" and dom.get("timer") == 15000
          and dom.get("acc") == "36px"
          and dom.get("transform", "").startswith("translate(-")
          and dom.get("staleBody") == "off"
          and dom.get("staleLive") == "OFFLINE &middot; 0 S AGO"
          and dom.get("nofix") == "map nofix" and dom.get("timer2") == 30000
          and dom.get("brokenBody") == "off" and dom.get("never") == "NO ANSWER",
          repr(dom))
    hd = v.get("hist", {})
    p1 = hd.get("path1", "")
    p2 = hd.get("path", "")
    check("PAGE-HIST: the day history is asked after the first status, "
          "from byte 0, then only from where it got to",
          dom.get("hist") == ["/hist?d=D&o=0"]
          and hd.get("urls2") == ["/hist?d=D&o=%s" % hd.get("cut")]
          and not hd.get("busy1") and hd.get("again"), repr(hd)[:600])
    check("PAGE-HIST: lines since local midnight, a cut line kept for the "
          "next reply, a gap over 5 min breaks the path",
          hd.get("pts1") == 10 and p1.count("M") == 1 and p1.count("L") == 9
          and hd.get("pts") == 11 and p2.count("M") == 2
          and p2.startswith('<svg width="1" height="1"><path d="M'),
          repr((hd.get("pts1"), hd.get("pts"), p1[:80], p2[-80:])))
    check("PAGE-HIST: zoom fitted to the track (15-17), layer moves with the "
          "tiles, TODAY km in the HUD",
          15 <= (hd.get("zoom") or 0) <= 16 and hd.get("trkT")
          and hd.get("day") == "<em>TODAY </em>%.2f KM" % hd.get("km", -1)
          and 0.28 < hd.get("km", 0) < 0.32, repr((hd.get("zoom"),
                                                    hd.get("day"))))
    check("PAGE: satellite table and map tile maths",
          "<td>SBAS 5</td><td>-</td><td>-</td><td>-</td>" in v.get("svt", "")
          and v.get("svh") == "Satellite table &middot; 3"
          and v.get("tile", {}).get("x") == 72235
          and v.get("tile", {}).get("y") == 46048, repr(v.get("tile")))
with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                       "web.html")) as f:
    page_src = f.read()
check("PAGE: polls status.json at the board's refresh_s, no reload",
      "/status.json" in page_src and "refresh_s" in page_src
      and 'http-equiv="refresh"' not in page_src
      and "tile.openstreetmap.org" in page_src and "OpenStreetMap</a>" in page_src
      and '<link rel="icon" href="data:image/svg+xml,' in page_src
      and "el.offsetWidth / 512" in page_src and "window.onresize" in page_src
      and 'name="viewport"' in page_src)


# MINMOVE-ENV: .env MIN_MOVE 0-50 m (decimals, comment), anything else keeps
saved_mm = gl.MIN_MOVE
got = []
for v in ("2.5", "0", "7 # meters", "60", "-1", "x", None):
    gl.MIN_MOVE = 1
    gl._min_move_env({"MIN_MOVE": v} if v is not None else {})
    got.append(gl.MIN_MOVE)
gl.MIN_MOVE = saved_mm
check("MINMOVE-ENV: 0-50 m read (decimals, comment); invalid keeps 1",
      got == [2.5, 0.0, 7.0, 1, 1, 1, 1], repr(got))

print()
print("%d passed, %d failed" % (passed, failed))
if failures:
    print("failures:")
    for f in failures:
        print(" ", f)
sys.exit(1 if failed else 0)
