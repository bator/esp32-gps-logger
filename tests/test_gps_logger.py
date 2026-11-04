#!/usr/bin/env python3
"""CPython harness for gps_logger upload / crash / WiFi logic. Not deployed."""
from __future__ import print_function

import builtins
import calendar
import gc as real_gc
import hashlib
import importlib
import importlib.util
import io
import json
import math
import os
import re
import shutil
import sys
import tempfile
import time
import stat
import types
import zlib

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)


def _gz_small(data, wbits):
    """A gzip of data with a 2**wbits window (as deploy.sh and
    build_firmware.sh make the page now)."""
    z = zlib.compressobj(9, zlib.DEFLATED, 16 + wbits)
    return z.compress(data) + z.flush()


def _gunzip_w(data, wbits):
    """data inflated with a 2**wbits window, as the board does it (a gzip
    made with a wider one fails: a distance too far back), or None."""
    try:
        z = zlib.decompressobj(16 + wbits)
        out = z.decompress(data)
        return out if z.eof and not z.unused_data else None
    except (zlib.error, TypeError):
        return None


class _FakeDeflate:
    """MicroPython 1.29's deflate module over zlib, for CPython: DeflateIO
    reads a gzip stream with a 2**wbits window, as uzlib does (a stream
    made with a wider one fails). made: the wbits each was opened with."""
    GZIP = 3
    made = []

    class DeflateIO:
        def __init__(self, f, fmt, wbits=0, close=False):
            assert fmt == 3, fmt
            _FakeDeflate.made.append(wbits)
            self.f, self.left = f, b""
            self.z = zlib.decompressobj(16 + (wbits or 8))

        def readinto(self, b):
            while len(self.left) < len(b) and not self.z.eof:
                d = self.f.read(256)
                if not d:
                    break
                self.left += self.z.decompress(d)
            n = min(len(b), len(self.left))
            b[:n] = self.left[:n]
            self.left = self.left[n:]
            return n

        def close(self):
            pass


class _LowMemDeflate(_FakeDeflate):
    """deflate whose window does not fit (MemoryError at the first read,
    where uzlib allocates it)."""
    class DeflateIO(_FakeDeflate.DeflateIO):
        def readinto(self, b):
            raise MemoryError("window")

# New tracks are binary .trb files. The checks here read a track's text:
# a .trb opened for reading by this file (not by the logger) comes back as
# the text lines it holds, decoded by tools/trk2gpx.py (written apart from
# the logger's own decoder, so the two check each other), with the header
# the text file would have had.
_t2g_spec = importlib.util.spec_from_file_location(
    "trk2gpx", os.path.join(ROOT, "tools", "trk2gpx.py"))
trk2gpx = importlib.util.module_from_spec(_t2g_spec)
_t2g_spec.loader.exec_module(trk2gpx)


def trb_text(data):
    """The text a .trb's bytes stand for, "#TRK1"/"#TRK2" header first."""
    lines = list(trk2gpx.trb_lines(data))
    if lines and lines[0] == "#TRKB\n":
        pts = [x for x in lines if x[:1] not in ("#", "-")]
        lines[0] = "#TRK2\n" if pts and pts[0].count(",") == 5 else "#TRK1\n"
    return "".join(lines)


def open(path, mode="r", *args, **kw):    # noqa: A001 (see above)
    p = path if isinstance(path, str) else None
    if p is not None and p.endswith(".trb") and mode in ("r", "rt", "rb"):
        with builtins.open(p, "rb") as f:
            text = trb_text(f.read())
        return io.BytesIO(text.encode()) if "b" in mode else io.StringIO(text)
    return builtins.open(path, mode, *args, **kw)

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
        if "value" in k:
            _pin_levels.append((n, k["value"], _clock[0]))

    def value(self, v=None):
        if v is None:
            return 1
        self._v = v
        _pin_levels.append((self.n, v, _clock[0]))
        return None

    def __call__(self, v=None):
        return self.value(v)


class I2C(object):
    def __init__(self, *a, **k):
        pass


class _TftSpi(object):
    """The TFT's SPI and the panel behind it: commands (DC low) and data (DC
    high) as an ST7735S takes them, in the controller's coordinates."""
    out = []
    fail = False
    panel = {}
    win = [0, 0, 0, 0]
    cmd = None
    frames = 0

    def __init__(self, *a, **k):
        pass

    def write(self, data):
        if _TftSpi.fail:
            raise OSError(5)
        data = bytes(data)
        dc = next((v for n, v, _t in reversed(_pin_levels) if n == 40), 0)
        if not dc:
            _TftSpi.cmd = data[0]
            _TftSpi.out.append(data)
            return
        c = _TftSpi.cmd
        _TftSpi.out.append(data)
        w = _TftSpi.win
        if c == 0x2A:
            w[0], w[2] = data[0] << 8 | data[1], data[2] << 8 | data[3]
        elif c == 0x2B:
            w[1], w[3] = data[0] << 8 | data[1], data[2] << 8 | data[3]
        elif c == 0x2C:
            _TftSpi.frames += 1
            cols = w[2] - w[0] + 1
            for i in range(0, len(data), 2):
                n = i // 2
                _TftSpi.panel[(w[0] + n % cols, w[1] + n // cols)] = data[i] | data[i + 1] << 8


class _FakeFramebuf(object):
    """framebuf as extmod/modframebuf.c does it, for the formats gps_tft uses:
    RGB565 stored little endian in the buffer, MONO_HLSB, blit from a
    FrameBuffer or a (buffer, w, h, format) tuple, read-only buffers too, with
    key and palette, clipped on all sides."""
    RGB565, MONO_HLSB, GS4_HMSB = 1, 3, 2

    class FrameBuffer(object):
        def __init__(self, buf, w, h, fmt):
            need = (w * h * 2 if fmt == 1 else ((w + 7) // 8) * h if fmt == 3
                    else ((w + 1) // 2) * h)
            if len(buf) < need:
                raise ValueError("buffer too small")
            self.buf, self.w, self.h, self.fmt = buf, w, h, fmt

        def pixel(self, x, y, c=None):
            if self.fmt == 1:
                o = (x + y * self.w) * 2
                if c is None:
                    return self.buf[o] | self.buf[o + 1] << 8
                self.buf[o], self.buf[o + 1] = c & 255, c >> 8
                return None
            if self.fmt == 2:
                o = ((self.w + 1) // 2) * y + x // 2
                return self.buf[o] & 15 if x % 2 else self.buf[o] >> 4
            o = ((self.w + 7) // 8) * y + x // 8
            return self.buf[o] >> (7 - x % 8) & 1

        def fill_rect(self, x, y, w, h, c):
            for j in range(max(y, 0), min(y + h, self.h)):
                for i in range(max(x, 0), min(x + w, self.w)):
                    self.pixel(i, j, c)

        def fill(self, c):
            self.fill_rect(0, 0, self.w, self.h, c)

        def blit(self, src, x, y, key=-1, palette=None):
            if isinstance(src, tuple):
                src = _FakeFramebuf.FrameBuffer(src[0], src[1], src[2], src[3])
            for j in range(max(0, -y), min(src.h, self.h - y)):
                for i in range(max(0, -x), min(src.w, self.w - x)):
                    c = src.pixel(i, j)
                    if palette is not None:
                        c = palette.pixel(c, 0)
                    if c != key:
                        self.pixel(x + i, y + j, c)


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
    machine.SPI = _TftSpi
    machine.UART = UART
    sys.modules["framebuf"] = _FakeFramebuf
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
            if n.endswith((".gpx", ".trk", ".trb", ".pos", ".tmp", ".bad")):
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
    # a fix lost while the flash is full is noted then, not when space
    # returns: NEW_FILE_GAP counts from the loss
    gl.free = lambda: 100
    gl.reset()                               # (no fix)
    gl.log.update(full=True, full_check=None, fix=True, lost_at=None)
    _clock[0] += 1000
    lost_t = _clock[0]
    gl.log_tick()
    _clock[0] += gl.NEW_FILE_GAP * 1000
    gl.free = lambda: gl.MIN_FREE + 10
    gl.log_tick()
    check("full: a fix lost while full sets lost_at at the loss",
          gl.log["full"] is False and gl.log["fix"] is False
          and gl.log["lost_at"] == lost_t,
          "fix=%s lost_at=%s want %s" % (gl.log["fix"], gl.log["lost_at"],
                                        lost_t))
    gl.free = orig_free

    # FULL: a full flash drops the batch and records nothing until space
    # returns. The track then goes on in a new segment (before, a 1221 s
    # gap had no "-" and the next line measured MIN_MOVE from a stale
    # prev), or in a new file after NEW_FILE_GAP, as after a fix loss. The
    # dropped lines are counted and the full spell noted in the diag log.
    full_dir = os.path.join(tmpdir, "full")
    os.mkdir(full_dir)
    saved_ht = gl._hist_trim
    full_room = [10 << 20]
    gl.free = lambda: full_room[0]
    gl._hist_trim = lambda want: full_room[0]
    full_r = []
    try:
        for full_s in (60, 1200):
            gl.LOG_DIR = full_dir
            for n in os.listdir(full_dir):
                os.remove(os.path.join(full_dir, n))
            try:
                os.remove(gl.DIAG_FILE)
            except OSError:
                pass
            gl.reset()
            gl.log_reset()
            gl.wifi_reset()
            gl.wifi.update(state="offline", home=False)
            k = 0
            for part in (200, full_s, 200):
                full_room[0] = 1000 if part == full_s else 10 << 20
                for _ in range(part):
                    _clock[0] += 1000
                    gl.st.update(valid=True, date="2026-10-18",
                                 time="%02d:%02d:%02d" % (
                                     8 + k // 3600, k // 60 % 60, k % 60),
                                 lat=47.5, lon=19.0 + k * 10.0 / 75000,
                                 alt=100.0, pos_at=_clock[0], hdop=1.0,
                                 hdop_at=_clock[0], status="A", quality=1,
                                 fixtype=3, speed=36.0)
                    gl.log_tick()
                    k += 1
            gl.flush()
            unbroken = 0
            files = sorted(n for n in os.listdir(full_dir)
                           if n.endswith((".trk", ".trb")))
            for n in files:
                prev_s = None
                with open(os.path.join(full_dir, n)) as f:
                    for l in f:
                        if l == gl.NEW_SEG:
                            prev_s = None
                        elif l.count(",") == 4:
                            tc = l.strip().split(",")[4]
                            s = (int(tc[9:11]) * 3600 + int(tc[11:13]) * 60
                                 + int(tc[13:15]))
                            if prev_s is not None and s - prev_s > 5:
                                unbroken += 1
                            prev_s = s
            dl = []
            try:
                with open(gl.DIAG_FILE) as f:
                    dl = [l for l in f if l.startswith("track full")]
            except OSError:
                pass
            full_r.append((full_s, len(files), unbroken,
                           gl.log.get("drop", 0) > 0, len(dl)))
    finally:
        gl.free = orig_free
        gl._hist_trim = saved_ht
        gl.LOG_DIR = tmpdir
        gl.log_reset()
    check("FULL: after a full flash the track goes on in a new segment (a "
          "new file past NEW_FILE_GAP), the drop counted and in the diag log",
          full_r == [(60, 1, 0, True, 1), (1200, 2, 0, True, 1)],
          repr(full_r))

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
    # while the scan shows some blank AP (WB-5: none, a hidden one cannot
    # be there either)
    def wr_rounds(env_text, n, aps=(("unrelated", -40),)):
        start_wifi(env_text, list(aps), [])
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
    hid = wr_rounds(env_hid_home, 2, (("unrelated", -40), ("", -80)))
    hid_none = wr_rounds(env_hid_home, 2)
    check("WR-a: unlisted networks are tried at boot and every Nth round",
          tries == [[]] * (every - 1) + [["home", "phone", "cabin"]]
          and hid == [["home"], ["home"]] and hid_none == [[], []],
          "%r %r %r" % (tries, hid, hid_none))
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
        if n.endswith((".gpx", ".trk", ".trb", ".pos", ".tmp", ".bad")):
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
    check("TRK-1: new file is a .trb (binary) under LOG_DIR",
          gl.log["file"] is not None and gl.log["file"].endswith(".trb")
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
          ok_bad is None and bad.count(b"\n") == 3 and b"#" not in bad,
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

    def _strict_json(b):
        """Parsed JSON, or the error: no inf / nan (invalid JSON, a 400 and
        the whole batch quarantined)."""
        def bad(c):
            raise ValueError(c)
        try:
            return json.loads(b.decode(), parse_constant=bad)
        except ValueError as e:
            return repr(e)

    # UPL-5: a finite hdop whose horizontal accuracy (* HDOP_M) is not
    # finite is left out, as an inf hdop is (it went into the JSON as inf)
    inf_p = os.path.join(end_dir, "20261005_170000.trk")
    with open(inf_p, "wb") as f:
        f.write((gl.HEADER + "47.100000,19.100000,100.0,1e308,"
                 "20261101T100000Z\n" + gl.FOOTER).encode())
    n, n_ok, size, new_off, eof = gl.read_batch(inf_p, 0)
    j_trk = _strict_json(bytes(gl._body[:size]))
    inf_g = os.path.join(end_dir, "20261005_170000.gpx")
    make_gpx(inf_g, 1, extras=["<ele>1.0</ele><hdop>1e308</hdop>"])
    n, n_ok, size, new_off, eof = gl.read_batch(inf_g, 0)
    j_gpx = _strict_json(bytes(gl._body[:size]))
    check("UPL-5: hdop * HDOP_M not finite: no accuracy, valid JSON, the "
          "point kept (.trk and .gpx)",
          all(isinstance(j_, dict) and len(j_["locations"]) == 1
              and "horizontal_accuracy" not in j_["locations"][0]
              ["properties"] for j_ in (j_trk, j_gpx)),
          repr((j_trk, j_gpx))[:300])

    # UPL-6: a point time that is no UTC time (month 13, day 32, Feb 30,
    # hour 24, minute 60, second 60) is no point, as _kb_time has it
    bad_t = ("20261301T100000Z", "20261032T100000Z", "20260230T100000Z",
             "20261101T240000Z", "20261101T106000Z", "20261101T100060Z",
             "20261100T100000Z", "20240229T100000Z", "20990101T000000Z")
    tm_p = os.path.join(end_dir, "20261005_180000.trk")
    with open(tm_p, "wb") as f:
        f.write((gl.HEADER + "".join("47.1,19.1,1.0,1.0,%s\n" % t_
                                     for t_ in bad_t) + gl.FOOTER).encode())
    n, n_ok, size, new_off, eof = gl.read_batch(tm_p, 0)
    j_tm = _strict_json(bytes(gl._body[:size]))
    tg_p = os.path.join(end_dir, "20261005_180000.gpx")
    iso_t = ["%s-%s-%sT%s:%s:%sZ" % (t_[0:4], t_[4:6], t_[6:8], t_[9:11],
                                     t_[11:13], t_[13:15]) for t_ in bad_t]
    make_gpx(tg_p, len(iso_t), times=iso_t)
    n2, n2_ok, size2, new_off, eof = gl.read_batch(tg_p, 0)
    j_tg = _strict_json(bytes(gl._body[:size2]))
    want_t = ["2024-02-29T10:00:00Z", "2099-01-01T00:00:00Z"]
    got_t = [[x["properties"]["timestamp"] for x in j_["locations"]]
             if isinstance(j_, dict) else j_ for j_ in (j_tm, j_tg)]
    check("UPL-6: a point time out of range (month, day, hour, minute, "
          "second) is skipped in a .trk and a .gpx",
          (n, n_ok, n2, n2_ok) == (9, 2, 9, 2) and got_t == [want_t] * 2,
          repr((n, n_ok, n2, n2_ok, got_t)))

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
    # the firmware's own upload body (it was a JSON the test built itself)
    k5_path = os.path.join(tmpdir, "20261005_23k5.trk")
    make_trk(k5_path, 3)
    k5_n, k5_ok, k5_size = gl.read_batch(k5_path, 0)[:3]
    try:
        k5_locs = json.loads(bytes(gl._body[:k5_size]))["locations"]
    except (ValueError, KeyError, TypeError) as e:
        k5_locs = repr(e)
    os.remove(k5_path)
    check("K5: the default device_id gives a valid upload body",
          isinstance(k5_locs, list) and len(k5_locs) == 3 == k5_ok
          and all(p_["properties"]["device_id"] == "Dawarich-GPS-tracker"
                  for p_ in k5_locs), repr(k5_locs)[:300])

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
    disk = [n for n in os.listdir(drive_dir) if n.endswith((".trk", ".trb"))]
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
    park_disk = [n for n in os.listdir(park_dir) if n.endswith((".trk", ".trb"))]
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
    hot_disk = [n for n in os.listdir(hot_dir) if n.endswith((".trk", ".trb"))]
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

    # NMEA-DR: GGA quality 6 and RMC mode E are an estimate (dead
    # reckoning, the last fix carried on) and were a fix; NAV-PVT's
    # DR-only fix is not one (PVT-FILTER). DGPS (2) still is
    def dr_case(q, mode, gga=True, rmc=True):
        gl.reset()
        for t in ("120040.00", "120041.00", "120042.00"):
            if rmc:
                gl.handle(nmea("GPRMC,%s,A,4730.000,N,01900.000,E,0.1,0.0,"
                               "171026,,%s" % (t, "" if mode is None
                                                else "," + mode)))
            if gga:
                gl.handle(nmea("GPGGA,%s,4730.000,N,01900.000,E,%d,04,2.0,"
                               "100.0,M,40.0,M,," % (t, q)))
        return gl.st["valid"], gl.st["seq"]

    dr = (dr_case(6, "E"), dr_case(6, "A", rmc=False),
          dr_case(1, "E", gga=False), dr_case(2, "D"), dr_case(1, "A"),
          dr_case(1, None, gga=False))       # (NMEA 2.2: no mode field)
    check("NMEA-DR: GGA quality 6 / RMC mode E -> no fix; 1, 2 still are",
          dr == ((False, 0), (False, 0), (False, 0), (True, 2), (True, 2),
                 (True, 2)), repr(dr))

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
    # REV-5: at 1 Hz GST comes once per 5 epochs (and one may be lost):
    # the seconds between keep its estimate for GST_HOLD, not None (that
    # let 4 of 5 seconds of a converging cold start through)
    gl.reset()
    gst_second("120130.00", 150.0)
    _clock[0] += 1000
    epoch("120131.00")
    _clock[0] += 1000
    epoch("120132.00")
    held = (gl.st["std"], gl._usable(_clock[0]))
    _clock[0] += getattr(gl, "GST_HOLD", 6000) + 1000
    epoch("120133.00")
    _clock[0] += 1000
    epoch("120134.00")
    check("REV-5: a second without GST keeps the last estimate (GST_HOLD), "
          "then only HDOP decides",
          held == (150.0, False) and gl._usable(_clock[0]),
          repr((held, gl.st.get("std"))))

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
    check("WIFI-202: a weak candidate (< ROAM_MIN_RSSI) failing with 202 "
          "(STAT_WRONG_PASSWORD, the C6 quirk) is retried once",
          calls == ["home", "home", "phone"] and w["ssid"] == "phone",
          "calls=%s" % calls)
    # the mock fails with 4 (its STAT_WRONG_PASSWORD): as another status
    # (beacon timeout, no AP) the weak candidate gets no retry
    saved_wp = sys.modules["network"].STAT_WRONG_PASSWORD
    sys.modules["network"].STAT_WRONG_PASSWORD = 99
    try:
        w, calls, out = rv_start_wifi(rv_env, [("home", -80), ("phone", -85)],
                                      ["home", "phone"], fail_always=("home",))
    finally:
        sys.modules["network"].STAT_WRONG_PASSWORD = saved_wp
    check("REVIEW3-2: a weak candidate failing otherwise is not retried",
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
            if n.endswith((".trk", ".trb")):
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
        # (drift past STILL_NEAR, here to 56 m, delays one by up to STILL_BACK)
        check("STAY: heartbeats are STILL_HEARTBEAT apart, at most STILL_BACK late",
              gl.STILL_HEARTBEAT // 1000 <= gap
              <= (gl.STILL_HEARTBEAT + gl.STILL_BACK) // 1000 + 2, "gap=%s" % gap)
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
    # ANTIMERID: across 180° the longitude difference wraps (the distance
    # was 38000 km, the window sums mixed +180 and -180): 10 min sitting
    # in Fiji is a stay, its drift dropped, its anchor a longitude there
    am_d = gl._dist(-17.0, 179.9999, -17.0, -179.9999)
    sv_reset()
    am_sec = rv_sec[0]                    # (later tests keep their times)
    arnd = random.Random(180)
    for i in range(600):
        lon = 179.9999 + arnd.uniform(-0.0002, 0.0002)
        if lon > 180:
            lon -= 360
        _clock[0] += gl.SAMPLE
        sec = rv_sec[0]
        rv_sec[0] += 1
        gl.st.update(valid=True, date="2026-10-18",
                     time="%02d:%02d:%02d" % (8 + sec // 3600,
                                              (sec // 60) % 60, sec % 60),
                     lat=-17.0 + arnd.uniform(-0.0001, 0.0001), lon=lon,
                     alt=10.0, pos_at=_clock[0], hdop=1.0,
                     hdop_at=_clock[0], status="A", quality=1, fixtype=3,
                     speed=0.5)
        gl.log_tick()
    gl.flush()
    am_l = sv_lines()
    am_a = gl.log["still"]["anchor"]
    am_lon = [float(l.split(",")[1]) for l in am_l]
    check("ANTIMERID: 21 m across 180°; a stay there, its anchor within "
          "+-180° and near the drift",
          20 < am_d < 22 and sv_active() and am_a is not None
          and -180 <= am_a[1] <= 180
          and gl._dist(am_a[0], am_a[1], -17.0, 179.9999) < 15
          and len(am_l) <= 2 * gl.STILL_WINDOW // 1000 + 10
          and all(-180 <= x <= 180 for x in am_lon),
          "d=%.0f active=%s anchor=%r lines=%d" % (
              am_d, sv_active(), am_a, len(am_l)))
    rv_sec[0] = am_sec
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

    # an out-and-back with the turn near a window boundary: the window out
    # and the window back have the same centroid. That paired as a stay and
    # dropped the held lines (a car lost 36 s / 360 m, a bike ~100 s).
    # (A walk: STAY-WALK below.)
    uturn = []
    for v in (10.0, 4.0, 3.0):
        for delta in range(-10, 11, 2):
            sv_reset()
            for i in range(180):
                sv_fix(i * v, 0, v * 3.6)
            x0 = 179 * v
            wa = gl.log["still"]["win_at"]
            to_b = gl.STILL_WINDOW - (gl._age(_clock[0], wa) if wa is not None
                                      else gl.STILL_WINDOW)
            turn = to_b // 1000 + gl.STILL_WINDOW // 1000 + delta
            stayed = False
            for t in range(1, turn + 1):
                sv_fix(x0, t * v, v * 3.6)
                stayed = stayed or sv_active()
            for t in range(1, 300):
                sv_fix(x0, (turn - t) * v, v * 3.6)
                stayed = stayed or sv_active()
            gl.flush()
            tcs = [l.strip().split(",")[4] for l in sv_lines()]
            secs = sorted(int(tc[9:11]) * 3600 + int(tc[11:13]) * 60
                          + int(tc[13:15]) for tc in tcs)
            gap = max(b - a for a, b in zip(secs, secs[1:]))
            if stayed or gap > 5:
                uturn.append((v, delta, stayed, gap))
    check("STAY: an out-and-back U-turn is no stay, nothing dropped",
          uturn == [], repr(uturn))
    # STAY-WALK: ... at walking pace too. Two windows of an out and back
    # (or a walk and the stop after it) have centroids within STILL_RADIUS
    # there: a 1.4 m/s walk 70-98 m out and back kept only 6-55 m of it,
    # at 2 m/s a 100-140 m detour went. A steady window (_walking) pairs
    # with nothing. (Own random numbers and times: later tests keep rnd's
    # sequence and rv_sec's hours.)
    def ws_secs(line, t0):
        tc = line.strip().split(",")[4]
        return (int(tc[9:11]) * 3600 + int(tc[11:13]) * 60 + int(tc[13:15])
                - 8 * 3600 - t0)

    ws_sec0 = rv_sec[0]
    wrnd = random.Random(1402)
    ws_bad = []
    for v, leg, lead in ((1.4, 50, 0), (1.4, 50, 40), (1.4, 60, 30),
                         (1.4, 70, 20), (1.2, 60, 10), (2.0, 50, 0),
                         (2.0, 60, 40), (2.0, 70, 30)):
        sv_reset()
        rv_sec[0] = 0
        stayed = False
        for i in range(200 + lead):
            sv_fix(-v * (200 + lead) + i * v, wrnd.gauss(0, 1.5), v * 3.6)
        for i in range(1, leg + 1):
            sv_fix(i * v, wrnd.gauss(0, 1.5), v * 3.6)
            stayed = stayed or sv_active()
        for i in range(1, 300):
            sv_fix((leg - i) * v, wrnd.gauss(0, 1.5), v * 3.6)
            stayed = stayed or sv_active()
        gl.flush()
        ws_l = sv_lines()
        far = max((float(l.split(",")[1]) - 19.0) / M_LON for l in ws_l)
        secs = sorted(ws_secs(l, 0) for l in ws_l)
        gap = max(b - a for a, b in zip(secs, secs[1:]))
        if stayed or far < 0.9 * leg * v or gap > 5:
            ws_bad.append((v, leg, lead, stayed, round(far), gap))
    check("STAY-WALK: a walking out and back (1.2-2 m/s) is no stay, its "
          "turn kept", ws_bad == [], repr(ws_bad))
    # ... and a walk, a stop of 2.5-10 min, a walk on: the arrival was cut
    # at STILL_NEAR from an anchor pulled back along the way in (~50 s /
    # 69 m lost), and the departure ring held the last STILL_DEP_MAX
    # samples of a walk out past STILL_EXIT, ~120 s (lost: ~25 s / 37 m)
    ws_bad = []
    for pause, off in ((150, 0), (150, 20), (150, 40), (300, 10),
                       (300, 30), (600, 50)):
        sv_reset()
        rv_sec[0] = 0
        stayed = False
        t_in = 300 + off
        for i in range(t_in):
            sv_fix((i - t_in) * 1.4, wrnd.gauss(0, 0.7), 5.0)
        for i in range(pause):
            sv_fix(wrnd.gauss(0, 3), wrnd.gauss(0, 3), 0.3)
            stayed = stayed or sv_active()
        for i in range(1, 300):
            sv_fix(i * 1.4 + wrnd.gauss(0, 0.7), wrnd.gauss(0, 0.7), 5.0)
        gl.flush()
        ws_l = [l for l in sv_lines() if l.split(",")[2] != ""]
        arr = t_in - max([ws_secs(l, 0) for l in ws_l
                          if ws_secs(l, 0) < t_in] or [0])
        dep = min([ws_secs(l, 0) for l in ws_l
                   if ws_secs(l, 0) > t_in + pause] or [9999]) - t_in - pause
        if not stayed or arr > 8 or dep > 8:
            ws_bad.append((pause, off, stayed, arr, dep))
    check("STAY-WALK: walking in to a stop and on, at most a few seconds of "
          "the walk are lost on either side", ws_bad == [], repr(ws_bad))
    rv_sec[0] = ws_sec0

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
              new[0].strip().split(",")[4].replace("T", "_")[:15] + gl.TRB),
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

    # TIME-BACK: one plausible but false time (a receiver glitch, 3 h on)
    # became last_t, and every later line, heartbeats too, was older:
    # nothing was written until a reboot. Usable fixes that stay over an
    # hour behind it for TIME_BACK_N seconds let it go: a new segment, the
    # false line still in RAM dropped, a diag line.
    def tb_walk(k0, n, hh=8):
        for k in range(k0, k0 + n):
            _clock[0] += gl.SAMPLE
            gl.st.update(valid=True, date="2026-10-18",
                         time="%02d:%02d:%02d" % (hh, k // 60, k % 60),
                         lat=47.5, lon=19.0 + k * 1.4 * M_LON, alt=100.0,
                         pos_at=_clock[0], hdop=1.0, hdop_at=_clock[0],
                         status="A", quality=1, fixtype=3, speed=1.4)
            gl.log_tick()

    tb_saved = rnd.getstate(), rv_sec[0]     # (later tests keep theirs)
    sv_reset()
    try:
        os.remove(gl.DIAG_FILE)
    except OSError:
        pass
    tb_walk(0, 30)
    tb_walk(30, 1, hh=11)                    # the glitch: 3 h on
    tb_walk(31, 89)
    gl.flush()
    tb_l = sv_lines()
    tb_t = [l.strip().split(",")[4] for l in tb_l]
    tb_after = [x for x in tb_t if "T080031Z" <= x[8:] <= "T080159Z"]
    tb_txt = ""
    for n in sorted(os.listdir(rv_dir)):
        if n.endswith((".trk", ".trb")):
            with open(os.path.join(rv_dir, n)) as f:
                tb_txt += f.read()
    tb_d = []
    try:
        with open(gl.DIAG_FILE) as f:
            tb_d = [x for x in f if x.startswith("time back")]
    except OSError:
        pass
    check("TIME-BACK: a false future time lets go after TIME_BACK_N fixes "
          "an hour behind it: the walk goes on in a new segment",
          len(tb_after) >= 89 - getattr(gl, "TIME_BACK_N", 10) - 2
          and not any(x[9:11] == "11" for x in tb_t)
          and "\n-\n" in tb_txt and len(tb_d) == 1,
          "after=%d glitch=%s diag=%r" % (
              len(tb_after), [x for x in tb_t if x[9:11] == "11"], tb_d))
    # ... in a stay too, where no track line is sampled: the anchor
    # points come again
    sv_reset()
    sv_drift(150)
    tb_b0 = len([l for l in sv_lines() if l.split(",")[2] == ""])
    gl.log["last_t"] = "2026-10-18T23:00:00Z"
    sv_drift(400)
    tb_b1 = len([l for l in sv_lines() if l.split(",")[2] == ""])
    check("TIME-BACK: ... in a stay too, the anchor points come again",
          sv_active() and tb_b1 > tb_b0 and gl.log["last_t"] is not None
          and gl.log["last_t"] < "2026-10-18T23",
          "beats %d -> %d last_t=%s" % (tb_b0, tb_b1, gl.log["last_t"]))
    rnd.setstate(tb_saved[0])
    rv_sec[0] = tb_saved[1]

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

    # BEATPASS (2026-11-04, on the road): a walk to the car past STILL_NEAR held
    # the due anchor point back, the car then passed the anchor 44 m off, and
    # that second's anchor point, with the way out after it, read 158 km/h
    sv_reset()
    sv_drift(150, box=10.0, step=3.0)
    n0 = len(sv_lines())
    gl.log["still"]["hb_at"] = _clock[0] - gl.STILL_HEARTBEAT + 4000
    for i in range(60):
        sv_fix(90.0 + (i % 7), 30.0)               # at the car, 95 m out
    n1 = len(sv_lines())
    for i in range(45):
        sv_fix(90.0 - i * 8.0, -44.0, 30.0)        # the drive past, 8 m/s
    gl.flush()
    lines = sv_lines()
    step = []
    for a, b in zip(lines[n1:], lines[n1 + 1:]):
        dt = hb_secs(b) - hb_secs(a)
        step.append(gl._dist(*(map(float, a.split(",")[:2])),
                             *map(float, b.split(",")[:2])) / max(dt, 1))
    beats = [l for l in lines[n1:] if l.split(",")[2] == ""]
    check("BEATPASS: an anchor point due while out past STILL_NEAR waits; the "
          "car passing within STILL_NEAR writes none, no false speed on the way out",
          n1 == n0 and not beats and sv_active() is False
          and bool(step) and max(step) < 12.0,
          repr((n0, n1, len(beats), max(step) if step else None)))
    # ... and coming back to stay: the anchor points go on once it holds
    sv_reset()
    sv_drift(150, box=10.0, step=3.0)
    gl.log["still"]["hb_at"] = _clock[0] - gl.STILL_HEARTBEAT + 4000
    for i in range(30):
        sv_fix(90.0, 30.0)
    n1 = len(sv_lines())
    wait = None
    for i in range(gl.STILL_BACK // 1000 + 10):
        sv_fix(5.0, 5.0)
        if wait is None and len(sv_lines()) > n1:
            wait = i + 1
    check("BEATBACK: back at the place the anchor point comes after STILL_BACK",
          sv_active() is True and wait is not None
          and gl.STILL_BACK // 1000 - 1 <= wait <= gl.STILL_BACK // 1000 + 2,
          repr(wait))
    # ... and under drift out past STILL_NEAR all the time it is late, not lost
    sv_reset()
    sv_drift(150, box=10.0, step=3.0)
    gl.log["still"]["hb_at"] = _clock[0] - gl.STILL_HEARTBEAT
    n1 = len(sv_lines())
    wait = None
    for i in range(gl.STILL_HEARTBEAT // 1000 + 10):
        sv_fix(5.0 if i % 20 else 70.0, 5.0)
        if wait is None and len(sv_lines()) > n1:
            wait = i + 1
    check("BEATBACK: drift past STILL_NEAR every 20 s delays an anchor point by "
          "at most one STILL_HEARTBEAT",
          sv_active() is True and wait is not None
          and wait <= gl.STILL_HEARTBEAT // 1000 + 2, repr(wait))

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
        if n.endswith((".trk", ".trb")):
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
                 if n.endswith((".trk", ".trb")))
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
    # a fix that stays but is never usable (indoors, HDOP 9) closes no
    # window and is no fix loss: the lines held at the door stayed in RAM
    # for hours, and a reboot dropped them (88 lines). HOLD_RELEASE after
    # the last usable fix they go to flash, as after a fix loss.
    sv_reset()
    for i in range(180):
        sv_fix(i * 1.4, 0, 5.0)                     # a 3 min walk
    for i in range(30):
        sv_fix(252 + i % 3, 0, 0.0)                 # 30 s at the door
    held0 = len(gl.log["batch"])
    for _ in range((gl.HOLD_RELEASE + 2 * gl.FLUSH) // 1000):
        hb_fix(252, 0, hdop=9.0, speed=0.0)         # inside: not usable
    check("HOLD: a fix that stays unusable lets the held lines go after "
          "HOLD_RELEASE", held0 > gl.FLUSH // 1000 and not sv_active()
          and gl.log["batch"] == [] and len(sv_lines()) >= 200,
          "held=%d ram=%d lines=%d" % (held0, len(gl.log["batch"]),
                                       len(sv_lines())))

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
            if n.endswith((".trk", ".trb")):
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
          gl.log["file"].endswith("20261018_080010.trb")
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
    # DROP: lines the batch cap drops are counted (log["drop"], with the
    # full flash's): they went silently while writes failed
    check("DROP: the 61 lines the MAX_BATCH cap dropped are counted",
          gl.log.get("drop") == 61, "drop=%r" % gl.log.get("drop"))

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
             if n.endswith((".trk", ".trb")) and os.path.join(rv_dir, n) != trip
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
    inf_wbits = getattr(gl, "WEB_GZ_WBITS", 12)

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

    class SendLog(FakeConn):
        """A FakeConn that keeps the size of each send."""
        def __init__(self, req):
            FakeConn.__init__(self, req)
            self.lens = []

        def sendall(self, b):
            self.lens.append(len(b))
            self.sent += bytes(b)

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
        # WEB-INFLATE: the board keeps only web.html.gz (deploy.sh and the
        # OTA put no plain copy up): a client without gzip (curl, a test)
        # gets it inflated with the small window it was made with, a piece
        # per send; no deflate module is a 406, no RAM for it a 503; a
        # board still holding the old plain page serves it as before
        small_gz = _gz_small(page_file, inf_wbits)
        with open(os.path.join(gz_td, "small.gz"), "wb") as f:
            f.write(small_gz)
        gl.WEB_PAGE_GZ = os.path.join(gz_td, "small.gz")
        gl.WEB_PAGE = gl.WEB_PAGE + ".missing"
        saved_deflate = sys.modules.get("deflate", "unset")
        inf_res = {}
        try:
            for case, mod in (("plain", _FakeDeflate), ("gzip", _FakeDeflate),
                              ("nodeflate", None), ("lowmem", _LowMemDeflate)):
                sys.modules["deflate"] = mod
                del _FakeDeflate.made[:]
                c = SendLog(b"GET / HTTP/1.1\r\nHost: 192.168.1.60\r\n"
                            + (b"Accept-Encoding: gzip\r\n"
                               if case == "gzip" else b"") + b"\r\n")
                lst.waiting.append(c)
                gl.web_tick()
                inf_res[case] = (c, list(_FakeDeflate.made))
        finally:
            if saved_deflate == "unset":
                sys.modules.pop("deflate", None)
            else:
                sys.modules["deflate"] = saved_deflate
            gl.WEB_PAGE = gl.WEB_PAGE[:-len(".missing")]
        old_plain = SendLog(b"GET / HTTP/1.1\r\n\r\n")   # both on the board:
        lst.waiting.append(old_plain)                    # the plain one as is
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
    ic, iw = inf_res["plain"]
    ih, _, ib = ic.sent.partition(b"\r\n\r\n")
    check("WEB-INFLATE: only web.html.gz on the board: a client without "
          "gzip gets the page inflated (no Content-Length, no "
          "Content-Encoding), WEB_CHUNK a send, window WEB_GZ_WBITS",
          ih.startswith(b"HTTP/1.0 200 OK") and b"text/html" in ih
          and b"Content-Length" not in ih and b"Content-Encoding" not in ih
          and ib == page_file and iw == [inf_wbits]
          and max(ic.lens) <= gl.WEB_CHUNK and len(ic.lens) > 10
          and ic.closed, repr((ih[:200], len(ib), iw, ic.lens[:5])))
    gc_, gw = inf_res["gzip"]
    gh2, _, gb2 = gc_.sent.partition(b"\r\n\r\n")
    check("WEB-INFLATE: a gzip client still gets web.html.gz as is, nothing "
          "inflated", b"\r\nContent-Encoding: gzip\r\n" in gh2
          and gb2 == small_gz and gw == [], repr((gh2[:200], gw)))
    nd = inf_res["nodeflate"][0].sent
    lm = inf_res["lowmem"][0].sent
    check("WEB-INFLATE: no deflate module -> 406 naming curl --compressed; "
          "no RAM for the window -> 503; neither sends a 200",
          nd.startswith(b"HTTP/1.0 406") and b"--compressed" in nd
          and lm.startswith(b"HTTP/1.0 503") and b"200 OK" not in nd + lm,
          repr((nd[:200], lm[:200])))
    oh, _, ob = old_plain.sent.partition(b"\r\n\r\n")
    check("WEB-INFLATE: a board that still has the plain web.html (before "
          "its next deploy or OTA) sends it as is to a client without gzip",
          ob == page_file and (b"Content-Length: %d\r\n" % len(page_file))
          in oh, repr(oh[:200]))
    check("WEB-GZ-WBITS: the page's window is 2**9..2**12 bytes (the RAM "
          "the inflate takes: ~15 KB of heap is free; gzip -9's 32 KB is "
          "not)", 9 <= getattr(gl, "WEB_GZ_WBITS", 15) <= 12,
          getattr(gl, "WEB_GZ_WBITS", None))
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
        gl._keepalive_env({"KEEPALIVE_MODE": "Scan", "KEEPALIVE_PIN": "14"})
        m1, p1 = gl.PB_MODE, gl.PB_PIN
        gl._keepalive_env({"KEEPALIVE_MODE": "laser", "KEEPALIVE_PIN": "x"})
        check("PB_KEEPALIVE: .env KEEPALIVE_MODE / KEEPALIVE_PIN, bad ones kept",
              m1 == "scan" and p1 == 14 and gl.PB_MODE == "scan"
              and gl.PB_PIN == 14, "%s %s %s %s" % (m1, p1, gl.PB_MODE, gl.PB_PIN))
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
        # C6 strapping 4/5 and 8/9, UART0 16/17, SPI flash 24-30
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
            for v in (4, 5, 8, 9, 16, 17, 24, 27, 30):
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
with open(lb_gz, "wb") as f:              # (as deploy.sh makes it)
    f.write(_gz_small(lb_page, getattr(gl, "WEB_GZ_WBITS", 12)))
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
    check("WEB-LOOPBACK-INFLATE: a client without gzip gets web.html byte "
          "for byte from web.html.gz alone on " + tag,
          "ok   GET / without gzip, only web.html.gz" in loop_out,
          loop_out[-600:])
    check("WEB-LOOPBACK-DIAG: /diag?f=0 paged over a socket is diag.log "
          "byte for byte, a missing diag.1 a 404, on " + tag,
          "ok   GET /diag?f=0 paged from o is diag.log" in loop_out
          and "ok   GET /diag?f=1 without diag.1 is a 404" in loop_out,
          loop_out[-600:])
    if interp == mpy_bin:
        check("WEB-LOOPBACK-HEAP: the inflate's live heap stays small on "
              + tag, "ok   the inflate's live heap" in loop_out,
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
sys.implementation._machine = os.environ.get(
    "FAKE_MACHINE", "Generic ESP32C6 module with ESP32C6")
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
def logger(s):                  # what the logger does: a session's raw REPL
    with open(os.environ["FAKE_LOGGER"], "a") as f:      # stops it
        f.write(s + "\n")
logger("stop")
shim = open(os.environ["FAKE_SHIM"]).read()
for c in cmds:
    if c[:2] == ["exec", "--no-follow"]:
        logger("run " + c[2])   # (not run here: it would not return)
    elif c[0] == "reset":       # main.py runs it again; an OTA being
        logger("reset" + (" pending" if os.path.exists(    # validated aborts
            os.path.join(dev, "ota.pending")) else ""))
    elif c[0] == "exec":
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
                 ("web.html.dpl", b"left from a failed deploy")):
        with open(os.path.join(dev, n), "wb") as f:
            f.write(b)
    if env_extra.get("FAKE_OTA_PENDING"):      # an OTA install being validated:
        for n, b in (("ota.pending", b"1793557995 6b62646\n/web.html"),
                     ("web.html.new", b"page staged by the OTA")):
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
               FAKE_LOGGER=os.path.join(work, "logger"),
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


def dev_logger(dev):
    """What the fake board's logger went through: stop / reset / run ..."""
    try:
        with open(os.path.join(os.path.dirname(dev), "logger")) as f:
            return f.read().split("\n")[:-1]
    except OSError:
        return []


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
    check("DEPLOY: copies to .dpl, checks, renames; old .dpl and stray .py go",
          rc == 0 and dev_file(dev, "gps_logger.mpy") == new_mpy
          and dev_file(dev, "gps_logger.py") is None
          and not [n for n in os.listdir(dev) if n.endswith(".dpl")]
          and dev_file(dev, ".rtc") == b"0"
          and len(cps) == 3 and all(c[2].endswith(".dpl") for c in cps)
          and "Board: Generic ESP32C6" in out and "(rv32imc)" in out
          and "Deployed to /dev/fakeACM0" in out,
          "rc=%d %r %r %r" % (rc, out[-300:], err[-300:], cps))
    dp_w = getattr(gl, "WEB_GZ_WBITS", 12)
    check("DEPLOY-GZIP: only web.html.gz goes up, made with the board's "
          "small window (WEB_GZ_WBITS: it inflates it for curl), no mtime; "
          "the old plain web.html is removed in the same swap",
          _gunzip_w(dev_file(dev, "web.html.gz") or b"", dp_w) == new_page
          and (dev_file(dev, "web.html.gz") or b"")[4:8] == b"\0\0\0\0"
          and ["cp", "web.html.gz", ":web.html.gz.dpl"] in cps
          and not [c for c in cps if c[1] == "web.html"]
          and dev_file(dev, "web.html") is None, repr(cps))
    rc, out, err, dev, calls = deploy_run({"FAKE_FREE_BLOCKS": "3"})
    check("DEPLOY: too little free flash -> stops before copying, old kept",
          rc != 0 and dev_file(dev, "gps_logger.mpy") == b"OLD MODULE"
          and not [c for call in calls for c in call if c[0] == "cp"]
          and "Not enough free flash" in err, "rc=%d %r" % (rc, err[-300:]))
    rc, out, err, dev, calls = deploy_run({"FAKE_TRUNC": "gps_logger.mpy"})
    check("DEPLOY: a cut copy fails the check -> old files kept, .new removed",
          rc != 0 and dev_file(dev, "gps_logger.mpy") == b"OLD MODULE"
          and dev_file(dev, "main.py") == b"old main"
          and not [n for n in os.listdir(dev) if n.endswith(".dpl")]
          and "Copy check FAILED" in err
          and "Deployed" not in out, "rc=%d %r" % (rc, err[-300:]))
    rc, out, err, dev, calls = deploy_run({"FAKE_FROZEN": "match"})
    cps = [c for call in calls for c in call if c[0] == "cp"]
    check("DEPLOY: frozen firmware -> main.py and the page only, the .mpy goes",
          rc == 0 and dev_file(dev, "gps_logger.mpy") is None
          and dev_file(dev, "gps_logger.py") is None
          and dev_file(dev, "web.html") is None
          and _gunzip_w(dev_file(dev, "web.html.gz") or b"", dp_w) == new_page
          and sorted(c[1] for c in cps) == ["main.py", "web.html.gz"]
          and "frozen in (abc1234)" in out
          and "Deployed to /dev/fakeACM0" in out,
          "rc=%d %r %r %r" % (rc, out[-300:], err[-300:], cps))
    # DEPLOY-OTA: an OTA install still being validated (/ota.pending): the
    # deploy's reset would abort the pending image (the bootloader rolls back
    # and ota_boot blames the build), and its files would clash with the
    # staged page (<path>.new). It stops; FORCE=1 deploys, the staged page
    # left alone
    rc, out, err, dev, calls = deploy_run({"FAKE_OTA_PENDING": "1"})
    ota_stop = (rc != 0 and "OTA" in err
                and not [c for call in calls for c in call if c[0] == "cp"]
                and dev_file(dev, "web.html.new") == b"page staged by the OTA"
                and dev_file(dev, "main.py") == b"old main")
    rc, out, err, dev, calls = deploy_run({"FAKE_OTA_PENDING": "1",
                                           "FORCE_PENDING": "1"})
    check("DEPLOY-OTA: an OTA being validated stops the deploy before "
          "copying; FORCE_PENDING=1 deploys and leaves the staged page",
          ota_stop and rc == 0
          and gunzip(dev_file(dev, "web.html.gz") or b"") == new_page
          and dev_file(dev, "web.html.new") == b"page staged by the OTA",
          "%r rc=%d %r %r" % (ota_stop, rc, out[-200:], err[-300:]))
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
    # DEPLOY-RESTART: each mpremote session stops the logger (raw REPL);
    # the early exits left it stopped until the WDT reset the board (a boot
    # failure), and during an OTA validation that reset aborted the pending
    # image, the build blamed: what the /ota.pending stop is for. Every exit
    # restarts it: a reset, or with an OTA pending the logger run again
    # without one; the pending check is the first session
    run_cmd = "run import gps_logger; gps_logger.run()"
    rs = []
    for case, pend in ((({"FAKE_OTA_PENDING": "1"}), True),
                       ({"FAKE_OTA_PENDING": "1", "FORCE_PENDING": "1",
                         "FAKE_FREE_BLOCKS": "3"}, True),
                       ({"FAKE_FREE_BLOCKS": "3"}, False),
                       ({"FAKE_TRUNC": "gps_logger.mpy"}, False),
                       ({"FAKE_FROZEN": "stale"}, False),
                       ({"FAKE_GARBLE": "1"}, False),
                       ({"FAKE_MACHINE": "Raspberry Pi Pico with RP2040"},
                        False),
                       ({}, False)):
        rc, out, err, dev, calls = deploy_run(case)
        lg = dev_logger(dev)
        ok = (lg[-1:] == ([run_cmd] if pend else ["reset"])
              and "reset pending" not in lg and (rc == 0) == (case == {}))
        if pend and not case.get("FORCE_PENDING"):
            ok = ok and len(calls) == 2 and "ota.pending" in calls[0][0][1]
        rs.append(ok or (case, rc, lg, [[x[0] for x in c] for c in calls]))
    check("DEPLOY-RESTART: every exit restarts the logger: a reset, or with "
          "an OTA pending a run without one; the pending check comes first",
          rs == [True] * len(rs), repr(rs))
    # DEPLOY-FORCE-PENDING: FORCE_PENDING=1 during an OTA validation got
    # through the copy and then the last step's own reset aborted the
    # pending image (rolled back, the build blamed): no reset then, the
    # logger run again
    rc, out, err, dev, calls = deploy_run({"FAKE_OTA_PENDING": "1",
                                           "FORCE_PENDING": "1"})
    lg = dev_logger(dev)
    check("DEPLOY-FORCE-PENDING: FORCE_PENDING=1 with an OTA pending deploys "
          "without a reset and runs the logger again",
          rc == 0 and "Deployed" in out and "reset pending" not in lg
          and lg[-1:] == [run_cmd]
          and gunzip(dev_file(dev, "web.html.gz") or b"") == new_page,
          "rc=%d %r %r" % (rc, lg, err[-300:]))
    # DEPLOY-FORCE-SPLIT: one FORCE=1 meant for a stale frozen logger also
    # walked past the pending-OTA guard; that one is FORCE_PENDING=1 now
    rc, out, err, dev, calls = deploy_run({"FAKE_OTA_PENDING": "1",
                                           "FAKE_FROZEN": "stale",
                                           "FORCE": "1"})
    split = (rc != 0 and "FORCE_PENDING=1" in err
             and not [c for call in calls for c in call if c[0] == "cp"]
             and dev_file(dev, "main.py") == b"old main")
    rc, out, err, dev, calls = deploy_run({"FAKE_OTA_PENDING": "1",
                                           "FAKE_FROZEN": "stale",
                                           "FORCE_PENDING": "1"})
    split2 = (rc != 0 and "tools/build_firmware.sh --flash" in err
              and dev_file(dev, "main.py") == b"old main")
    check("DEPLOY-FORCE-SPLIT: FORCE=1 does not get past a pending OTA, "
          "FORCE_PENDING=1 does not get past a stale frozen logger",
          split and split2, "%r %r rc=%d %r" % (split, split2, rc, err[-300:]))
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
if "flash_id" in a:
    print("Detected flash size: " + os.environ.get("FAKE_FLASH", "16MB"))
    sys.stdout.flush()
    if os.environ.get("FAKE_FLASH_NOISE"):   # more output after it: a
        for i in range(20000):              # reader that stops early (grep
            print("noise %d %s" % (i, "x" * 40))   # -q) gives an EPIPE
"""
FAKE_SCP = r"""#!/usr/bin/env python3
import os, sys, json, shutil
a = [x for x in sys.argv[1:] if x != "-q"]
with open(os.environ["FAKE_LOG"], "a") as f:
    f.write(json.dumps(["scp"] + a) + "\n")
dst = a[-1].split(":", 1)[1] if ":" in a[-1] else a[-1]
for src in a[:-1]:
    shutil.copy(src, dst)
"""
FAKE_SSH = r"""#!/usr/bin/env python3
import os, sys, json, subprocess
a = sys.argv[1:]
with open(os.environ["FAKE_LOG"], "a") as f:
    f.write(json.dumps(["ssh"] + a) + "\n")
sys.exit(subprocess.run(["bash", "-c", " ".join(a[1:])]).returncode)
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
    shutil.copy(os.path.join(ROOT, "gps_boot.py"), work)
    shutil.copy(os.path.join(ROOT, "gps_tft.py"), work)
    shutil.copy(os.path.join(ROOT, "gps_tft_font.py"), work)
    shutil.copy(os.path.join(ROOT, "web.html"), work)
    fake = os.path.join(work, "fake")
    for d in ("bin", "idf", "mpy/mpy-cross", "mpy/ports/esp32"):
        os.makedirs(os.path.join(fake, d))
    os.makedirs(os.path.join(work, ".venv", "bin"))
    for path, src in (("fake/bin/esptool.py", FAKE_ESPTOOL),
                      ("fake/bin/scp", FAKE_SCP),
                      ("fake/bin/ssh", FAKE_SSH),
                      (".venv/bin/mpremote", FAKE_BUILD_MPREMOTE),
                      ("fake/idf/export.sh", ""),
                      ("fake/mpy/mpy-cross/Makefile", "all:\n\t@true\n"),
                      ("fake/mpy/ports/esp32/Makefile", FAKE_PORT_MK)):
        with open(os.path.join(work, path), "w") as f:
            f.write(src)
        os.chmod(os.path.join(work, path), 0o755)
    git = ["git", "-C", work, "-c", "user.name=t", "-c", "user.email=t@t"]
    subprocess.run(git + ["init", "-q"], check=True)
    subprocess.run(git + ["add", "gps_logger.py", "web.html"], check=True)
    cenv = dict(os.environ)
    if env_extra.get("FAKE_CT"):          # a fixed commit time (BUILD)
        cenv.update(GIT_COMMITTER_DATE="@%s +0000" % env_extra["FAKE_CT"],
                    GIT_AUTHOR_DATE="@%s +0000" % env_extra["FAKE_CT"])
    subprocess.run(git + ["commit", "-q", "-m", "x"], check=True, env=cenv)
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
        ct = int(subprocess.run(["git", "-C", work, "log", "-1",
                                 "--format=%ct"], capture_output=True,
                                text=True).stdout.strip())
        marks.append(rc == 0 and m.get("SHA") == sha and m.get("COMMIT")
                     == head + ("-dirty" if dirty else "")
                     and (m.get("BUILD") == ct if not dirty else
                          m.get("BUILD", 0) >= ct) and not calls)
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
    # BUILD-FLASH-FAIL: the write goes to ota_0 even when ota_0 runs: a
    # failed write can leave the board without a bootable app, not "as it
    # was" (the old claim). It says so and prints the way back
    with open(os.path.join(ROOT, "tools", "build_firmware.sh")) as f:
        bsrc = f.read()
    check("BUILD-FLASH-FAIL: a failed write prints the recovery (the backup, "
          "--flash again); the header no longer says the board is as it was",
          rc != 0 and "write_flash 0x10000" in err and "app-backup-" in err
          and "--flash again" in err
          and "failed check or write leaves the board as it was" not in bsrc,
          "rc=%d %r" % (rc, err[-400:]))
    # BUILD-SLIM: the board leaves out what the logger never uses (BLE,
    # ESP-NOW, LAN, mDNS queries, SD card, I2S: ~400 KB of the 2.02 MB app,
    # and the BLE controller's RAM) but keeps the mDNS responder (the page
    # as HOSTNAME.local: without it only the IP answered) and freezes only the port's own modules
    # with the logger (asyncio, umqtt, webrepl, mip, requests... ~25 KB
    # unused); the smaller app fits 1792 KB slots (2 x 2496 KB before: the
    # file system gets ~1.4 MB more on 16 MB, more of 8 MB)
    rc, out, err, work, calls = build_run([], {})
    with open(os.path.join(work, "firmware", "manifest.py")) as f:
        man = f.read()
    bd = os.path.join(ROOT, "boards", "GPS_LOGGER_C6")
    with open(os.path.join(bd, "mpconfigboard.h")) as f:
        bh = f.read()
    with open(os.path.join(bd, "mpconfigboard.cmake")) as f:
        bc = f.read()
    with open(os.path.join(bd, "sdkconfig.board")) as f:
        bs = f.read()
    with open(os.path.join(bd, "partitions-ota.csv")) as f:
        pt = [ln.split(",") for ln in f.read().splitlines()
              if ln.strip() and not ln.startswith("#")]
    pt = {r[0].strip(): (int(r[3], 16), int(r[4], 16)) for r in pt}
    with open(os.path.join(ROOT, "tools", "build_firmware.sh")) as f:
        app = re.search(r"^APP_SIZE=(0x[0-9a-f]+)", f.read(), re.M)
    app = int(app.group(1), 16) if app else None
    off = ["MICROPY_PY_BLUETOOTH", "MICROPY_PY_ESPNOW",
           "MICROPY_PY_NETWORK_LAN", "MICROPY_PY_MACHINE_SDCARD",
           "MICROPY_PY_MACHINE_I2S", "MICROPY_HW_ENABLE_MDNS_QUERIES"]
    check("BUILD-SLIM: no BLE/ESP-NOW/LAN/mDNS queries/SD/I2S, the mDNS "
          "responder on, only the port's and the logger's modules frozen, "
          "1792 KB slots the script knows",
          rc == 0 and "boards/manifest.py" not in man
          and 'freeze("$(PORT_DIR)/modules")' in man
          and all(re.search(r"#define %s\s+\(0\)" % m, bh) for m in off)
          and re.search(r"#define MICROPY_HW_ENABLE_MDNS_RESPONDER\s+\(1\)", bh)
          and "sdkconfig.ble" not in bc and "CONFIG_BT_ENABLED=n" in bs
          and "partitions-ota.csv" in bc
          and pt.get("ota_0") == (0x10000, 0x1c0000)
          and pt.get("ota_1") == (0x1d0000, 0x1c0000) and app == 0x1c0000,
          repr((rc, man[-200:], pt, app)))
    rc, out, err, work, calls = build_run(["--flash"], {})
    flash_build = frozen_marker(work).get("BUILD", 0)
    steps = [c[c.index("read_flash") + 1] if "read_flash" in c else
             "write" if "write_flash" in c else c[0] for c in calls]
    erase = [c for c in calls if "erase_region" in c]
    check("BUILD-FLASH: check, backup, write, otadata erased (the written "
          "slot boots); no REPL step; deploy.sh named",
          rc == 0
          and steps == ["0x8000", "0x10000", "0x1d0000", "write", "esptool"]
          and erase and erase[0][-2:] == ["0xd000", "0x2000"]
          and [c for c in calls if "read_flash" in c][1][-2] == "0x1c0000"
          and "./deploy.sh" in out
          and "write_flash 0x10000" in out and ", then ./deploy.sh" in out
          and "app1-backup-" in out.rstrip().splitlines()[-1]
          and flash_build >= time.time() - 300,
          "rc=%d %r %r %r" % (rc, steps, out[-300:], err[-200:]))
    # --migrate: only a 16 MB board; backup, erase, write the whole build
    rc, out, err, work, calls = build_run(["--migrate"], {"FAKE_FLASH": "4MB"})
    m4 = (rc, [c for c in calls if "erase_flash" in c or "write_flash" in c])
    rc, out, err, work, calls = build_run(["--migrate"], {})
    mig = [("read 0" if "read_flash" in c and c[c.index("read_flash") + 1]
            == "0" else "erase" if "erase_flash" in c else "write" if
            "write_flash" in c else "id" if "flash_id" in c else c[1])
           for c in calls]
    check("BUILD-MIGRATE: refused under 16 MB; else flash id, backup of the "
          "first 4 MB, erase, write of the whole build (@flash_args)",
          m4[0] != 0 and m4[1] == [] and rc == 0
          and mig == ["id", "read 0", "erase", "write"]
          and calls[-1][-1] == "@flash_args" and "put /.env" in out,
          repr((m4, mig, err[-200:])))
    # BUILD-MIGRATE-SECRET: the backup holds the old file system, /.env
    # (WiFi, API and OTA keys) too: it was mode 664
    bk = [n for n in os.listdir(os.path.join(work, "firmware"))
          if n.startswith("full-backup-")]
    modes = [stat.S_IMODE(os.stat(os.path.join(work, "firmware", n)).st_mode)
             for n in bk]
    check("BUILD-MIGRATE-SECRET: the full backup is mode 600 and said to "
          "hold secrets", rc == 0 and modes == [0o600] and "secret" in out,
          repr((modes, out[-300:])))
    # BUILD-MIGRATE-EPIPE: `esptool.py flash_id | grep -q` under pipefail:
    # grep stops at the match, esptool's next write gets EPIPE, the pipe
    # fails and a 16 MB board was refused
    rc, out, err, work, calls = build_run(["--migrate"],
                                          {"FAKE_FLASH_NOISE": "1"})
    check("BUILD-MIGRATE-EPIPE: more esptool output after the flash size "
          "line does not refuse a 16 MB board",
          rc == 0 and [c for c in calls if "erase_flash" in c],
          "rc=%d %r" % (rc, err[-300:]))
    # --publish: refused dirty or without a key; else a signed manifest the
    # board accepts, copied after the files
    key_dir = tempfile.mkdtemp(dir=dp_td)
    key_file = os.path.join(key_dir, "ota.key")
    bad_key = os.path.join(key_dir, "bad.key")
    with open(bad_key, "w") as f:
        f.write('"quoted key with spaces"\n')
    with open(key_file, "w") as f:
        f.write("0123456789abcdef0123456789abcdef\n")
    dest = tempfile.mkdtemp(dir=dp_td)
    penv = {"OTA_KEY_FILE": key_file, "OTA_DEST": "fakehost:" + dest}
    rc_dirty = build_run(["--publish"], penv, dirty=True)[0]
    rc_nokey = build_run(["--publish"], {"OTA_KEY_FILE": key_file + "x",
                                         "OTA_DEST": "fakehost:" + dest})[0]
    rc_badkey = build_run(["--publish"], {"OTA_KEY_FILE": bad_key,
                                          "OTA_DEST": "fakehost:" + dest})[0]
    rc, out, err, work, calls = build_run(["--publish"], penv)
    try:
        with open(os.path.join(dest, "manifest.json")) as f:
            man = json.load(f)
    except (OSError, ValueError):
        man = {}
    scp = [c for c in calls if c[0] == "scp"]
    ssh = [c for c in calls if c[0] == "ssh"]
    good = (man.get("hmac") == gl._hmac_hex(
        b"0123456789abcdef0123456789abcdef", gl._ota_signed(man))
        if man else False)
    named = [man.get("url")] + [f["url"] for f in man.get("files", [])]
    check("BUILD-PUBLISH: dirty, no key or a quoted/short key refused; else "
          "a manifest the board's HMAC check accepts, files named by commit "
          "(a board reading the old manifest still finds its files), the "
          "manifest copied last",
          rc_dirty != 0 and rc_nokey != 0 and rc_badkey != 0 and rc == 0
          and good and gl._ota_manifest_ok(man)
          and named[:1] == ["micropython-%s.bin" % man.get("commit")]
          and all(os.path.exists(os.path.join(dest, n)) for n in named),
          repr((rc_dirty, rc_nokey, rc_badkey, rc, man, scp, err[-200:])))
    # BUILD-PUBLISH-BOARD: a board other than the C6 names itself in the
    # manifest (signed); a folder holding another board's manifest is refused
    s3 = os.path.join(ROOT, "boards", "GPS_LOGGER_S3")
    dest3 = tempfile.mkdtemp(dir=dp_td)
    penv3 = {"OTA_KEY_FILE": key_file, "OTA_DEST": "fakehost:" + dest3,
             "BOARD_DIR": s3}
    rc3, _o, err3, _w, _c = build_run(["--publish"], penv3)
    try:
        with open(os.path.join(dest3, "manifest.json")) as f:
            man3 = json.load(f)
    except (OSError, ValueError):
        man3 = {}
    hmac3 = gl._hmac_hex(b"0123456789abcdef0123456789abcdef", gl._ota_signed(man3)) if man3 else None
    rc_c6on3 = build_run(["--publish"], {"OTA_KEY_FILE": key_file, "OTA_DEST": "fakehost:" + dest3})[0]
    rc_s3onc6, _o, err_c6, _w, _c = build_run(["--publish"], dict(penv3, OTA_DEST="fakehost:" + dest, FORCE="1"))
    check("BUILD-PUBLISH-BOARD: the Heltec's manifest names its board, signed; a "
          "board's folder takes no other board's firmware, FORCE=1 or not",
          rc3 == 0 and man3.get("board") == "GPS_LOGGER_S3"
          and man3.get("hmac") == hmac3 and "board" not in man
          and rc_c6on3 != 0 and rc_s3onc6 != 0 and "another board" in err_c6,
          repr((rc3, err3[-200:], man3, rc_c6on3, rc_s3onc6, err_c6[-200:])))
    # an unreadable manifest is not "the C6's": refused without FORCE, as any board's
    dest4 = tempfile.mkdtemp(dir=dp_td)
    with open(os.path.join(dest4, "manifest.json"), "w") as f:
        f.write("{cut off")
    penv4 = dict(penv3, OTA_DEST="fakehost:" + dest4)
    rc4, _o, err4, _w, _c = build_run(["--publish"], penv4)
    rc5, _o, err5, _w, _c = build_run(["--publish"], dict(penv4, FORCE="1"))
    with open(os.path.join(dest4, "manifest.json")) as f:
        man4 = f.read()
    check("BUILD-PUBLISH-CUT: an unreadable manifest is no other board's: refused as "
          "unreadable, and FORCE=1 can repair the folder",
          rc4 != 0 and "another board" not in err4 and "no readable build" in err4
          and rc5 == 0 and '"board": "GPS_LOGGER_S3"' in man4,
          repr((rc4, err4[-160:], rc5, err5[-160:], man4[:60])))
    rc6, _o, _e, work6, _c = build_run([], {"BOARD_DIR": s3})
    rc7, _o, _e, work7, _c = build_run([], {})
    with open(os.path.join(work6, "firmware", "manifest.py")) as f:
        m6 = f.read()
    with open(os.path.join(work7, "firmware", "manifest.py")) as f:
        m7 = f.read()
    fr6 = frozen_marker(work6)
    with open(os.path.join(work6, "gps_tft.py"), "rb") as f:
        tft6 = f.read()
    with open(os.path.join(work6, "gps_tft_font.py"), "rb") as f:
        tft6 += f.read()
    check("BUILD-TFT: the display's modules are frozen on the Heltec's firmware only, and "
          "its marker carries their SHA-256 (deploy.sh compares it)",
          rc6 == 0 and rc7 == 0 and 'module("gps_tft.py"' in m6 and "gps_tft" not in m7
          and fr6.get("TFT") == hashlib.sha256(tft6).hexdigest()
          and "TFT" not in frozen_marker(work7), repr((m6[-120:], m7[-120:], fr6)))
    # BUILD-PUBLISH-GZ: the board keeps only the gzip page: the manifest
    # names /web.html.gz alone, made with the board's small window (it
    # inflates it for a client without gzip); an older firmware (after a
    # rollback: OTA_PAGES of both pages, as on master) accepts it too
    with open(os.path.join(work, "web.html"), "rb") as f:
        pub_page = f.read()
    pub_gz = dev_file(dest, "web-%s.html.gz" % man.get("commit")) or b""
    saved_pages = gl.OTA_PAGES
    gl.OTA_PAGES = ("/web.html", "/web.html.gz")
    try:
        old_ok = gl._ota_manifest_ok(man)
    finally:
        gl.OTA_PAGES = saved_pages
    check("BUILD-PUBLISH-GZ: the manifest names only /web.html.gz "
          "(web-<commit>.html.gz, the board's WEB_GZ_WBITS window); no "
          "plain page published; an older firmware's check accepts it",
          rc == 0 and [f["path"] for f in man.get("files", [])]
          == ["/web.html.gz"]
          and named == ["micropython-%s.bin" % man.get("commit"),
                        "web-%s.html.gz" % man.get("commit")]
          and _gunzip_w(pub_gz, getattr(gl, "WEB_GZ_WBITS", 12)) == pub_page
          and not os.path.exists(os.path.join(
              dest, "web-%s.html" % man.get("commit"))) and old_ok,
          repr((man.get("files"), sorted(os.listdir(dest)), old_ok)))
    # BUILD-PUBLISH-ATOMIC: every file (the manifest last) goes up under a
    # .tmp name and is moved over the live one (a board mid-download of a
    # republished .bin, or reading a half-copied manifest, got a mix); a
    # build not newer than the published one is refused unless FORCE=1
    mvs = " ".join(" ".join(c[2:]) for c in ssh if "mv" in " ".join(c))
    order = [mvs.find(n + ".tmp") for n in named] + [
        mvs.find("manifest.json.tmp")]
    atomic = (rc == 0 and len(scp) == 3
              and all(c[-1].endswith(".tmp") for c in scp)
              and all(i >= 0 for i in order) and order[-1] == max(order)
              and not [n for n in os.listdir(dest) if n.endswith(".tmp")])
    ct = str(man.get("build"))
    rc_same, _o, err_same, _w, _c = build_run(["--publish"],
                                              dict(penv, FAKE_CT=ct))
    hi = json.dumps(dict(man, build=man.get("build", 0) + 1000))
    with open(os.path.join(dest, "manifest.json"), "w") as f:
        f.write(hi)
    rc_old = build_run(["--publish"], penv)[0]
    with open(os.path.join(dest, "manifest.json")) as f:
        kept = f.read() == hi
    rc_force = build_run(["--publish"], dict(penv, FORCE="1",
                                             FAKE_CT=ct))[0]
    with open(os.path.join(dest, "manifest.json")) as f:
        forced = json.load(f).get("build") == man.get("build")
    check("BUILD-PUBLISH-ATOMIC: files go up as .tmp and are moved in, the "
          "manifest last; republishing the same or an older build is refused "
          "unless FORCE=1",
          atomic and rc_same != 0 and "FORCE=1" in err_same and rc_old != 0
          and kept and rc_force == 0 and forced,
          repr((atomic, order, scp, ssh, rc_same, err_same[-200:], rc_old,
                kept, rc_force, forced)))
    # SECRETS-IGNORE: the migration recipe copied /.env and /hist into the
    # repo root, and neither was ignored; it uses ~/.config/gps-logger now
    ign = subprocess.run(
        ["git", "-C", ROOT, "check-ignore", "--no-index", "env.backup",
         "hist/20261101.trk", "x.env", ".env.new", "env.bak"],
        capture_output=True, text=True).stdout.split()
    tracked_ign = subprocess.run(
        ["git", "-C", ROOT, "ls-files", "-ci", "--exclude-standard"],
        capture_output=True, text=True).stdout.split()
    with open(os.path.join(ROOT, "README.md")) as f:
        rd = f.read()
    mig = rd[rd.index("**Once, over USB:**"):rd.index("**Then:**")]
    check("SECRETS-IGNORE: env.backup, hist/ and *.env* are ignored (no "
          "tracked file is); the migration recipe keeps them in "
          "~/.config/gps-logger, mode 600",
          len(ign) == 5 and tracked_ign == []
          and "~/.config/gps-logger" in mig and "chmod 600" in mig
          and "env.backup +" not in mig and "cp -r :/hist . " not in mig,
          repr((ign, tracked_ign, mig[:400])))
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

# UPL-6: trk2gpx keeps the points the logger uploads: a time out of range
# (year outside 2020-2099, month, day, hour, minute, second) is no point,
# and a field that is no number and not UTF-8 either (a damaged block)
# drops the point, a UTF-8 one only the field (it was read as U+FFFD and
# the point kept)
t6_dir = tempfile.mkdtemp(prefix="t2g6-")
t6_p = os.path.join(t6_dir, "20261101_100000.trk")
t6_rows = [b"47.1,19.1,1.0,1.0,%s\n" % t_ for t_ in (
    b"20261301T100000Z", b"20261032T100000Z", b"20260230T100000Z",
    b"20261101T240000Z", b"20261101T106000Z", b"20261101T100060Z",
    b"20191231T235959Z", b"21000101T000000Z", b"20240229T100000Z",
    b"20990101T000000Z")]
t6_rows += [b"47.2,19.2,\xff\xfe,1.0,20261101T100001Z\n",
            b"47.3,19.3,\xc3\xa9,1.0,20261101T100002Z\n",
            b"47.4,19.4,abc,,20261101T100003Z\n"]
with builtins.open(t6_p, "wb") as f:
    f.write(b"#TRK1\n" + b"".join(t6_rows) + b"#END\n")
t6_out = io.StringIO()
real_stdout = sys.stdout
sys.stdout = t6_out
try:
    t6_rc = trk2gpx.main(["trk2gpx.py", t6_p])
finally:
    sys.stdout = real_stdout
t6_times = [x.split("</time>")[0]
            for x in t6_out.getvalue().split("<time>")[1:]]
t6_n = gl.read_batch(t6_p, 0)
t6_fw = [x["properties"]["timestamp"] for x in json.loads(
    bytes(gl._body[:t6_n[2]]).decode())["locations"]]
shutil.rmtree(t6_dir, ignore_errors=True)
check("UPL-6: trk2gpx drops times out of range and a point with a field "
      "that is no number and not UTF-8, as the logger does",
      t6_rc == 0 and t6_times == t6_fw == [
          "2024-02-29T10:00:00Z", "2099-01-01T00:00:00Z",
          "2026-11-01T10:00:02Z", "2026-11-01T10:00:03Z"],
      repr((t6_times, t6_fw)))

# PAGE-THEME: one colour world (Bator picked "carbon"): every colour on
# the page is one of the theme's roles (no navy, cyan or graphite left
# from the panels' separate origins: 34 colours before); the map is the
# Atlas map as it is, in its own colours (Bator: "the Atlas map, not this
# black"), no filter on the tiles
with open(os.path.join(ROOT, "web.html")) as f:
    th_src = f.read()
th_hex = sorted(set(c.lower() for c in re.findall(r"#[0-9a-fA-F]{6}\b", th_src)))
check("PAGE-THEME: the carbon palette only (plus the map's navigation "
      "blue), no navy / cyan / graphite left, the Atlas map unfiltered",
      len(th_hex) <= 22
      and not [c for c in th_hex if c in ("#3b82f6",)]
      and not any(c in th_src for c in ("#22d3ee", "#060b16", "#16233d",
                                        "rgba(34,211,238", "#1d2622",
                                        "#d6e2ff"))
      and "hue-rotate" not in th_src and "invert(1)" not in th_src
      and re.search(r"\.tiles\{[^}]*filter", th_src) is None,
      repr(th_hex))

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


# GPS-SILENT: a receiver that sends nothing (the M10 with its RX held low
# goes silent until a power cycle) was only "No data!" on the screen and a
# banner on the page: the diag log, read after a walk, said nothing. Its
# start and its end are diag lines now, once each
diag_clear()
gl.reset()
_gs_t0 = _clock[0]
gl.st["start"] = _gs_t0
gl.gps_check(_clock[0] + gl.GPS_SILENT - 1000)
_gs_early = [l for l in diag_lines() if "gps silent" in l]
_clock[0] = _gs_t0 + gl.GPS_SILENT
gl.gps_check(_clock[0])
_clock[0] += 30000
gl.gps_check(_clock[0])
_gs_mute = [l for l in diag_lines() if "gps silent" in l]
gl.st["rx"] = _clock[0]                  # bytes again
gl.gps_check(_clock[0])
gl.gps_check(_clock[0] + 1000)
_gs_back = [l for l in diag_lines() if "gps back" in l]
check("GPS-SILENT: no bytes for GPS_SILENT is one diag line, their return "
      "one more", _gs_early == [] and len(_gs_mute) == 1
      and len(_gs_back) == 1 and "90 s" in _gs_back[0]
      and gl.st["mute_at"] is None, repr((_gs_early, _gs_mute, _gs_back)))
diag_clear()
gl.reset()


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
    gl._up[:] = [0, 0, 0]                # uptime from tick 0
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
    resets = {f: inspect.getsource(getattr(gl, f)) for f in (
        "_mem_reboot", "_ota_check", "_ota_health")}
    mem_via = all("_mem_reboot(" in inspect.getsource(getattr(gl, f))
                  for f in ("_tls_reboot", "_low_reboot"))
    gl._uptime_saved = gl.uptime_s
    diag_clear()
    gl.uptime_s = lambda now=None: 151
    try:
        gl._diag_write("up 65 utc 20261101T132141Z nmea 1/0 x")
        gl._diag_final()
        gl._diag_write("ota 86fb9c8 installed (was 8416d67), rebooting")
        pf, _, tf = prev_after(open(gl.DIAG_FILE).read())
    finally:
        gl.uptime_s = gl._uptime_saved
    check("UPTIME: a deliberate reset (OTA, TLS memory, low IDF, rollback) "
          "writes a status line first: the next boot sees the whole run",
          pf["up"] == 151 and pf["how"] == "status" and mem_via
          and all(src.index("_diag_final()") < src.index("machine.reset()")
                  for src in resets.values()), repr((pf, tf)))
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

# SETUP-CHECK-RESTART: a receiver restart within GPS_VERIFY (back at 1 Hz,
# its own setup comes) is no verdict on the setup before it in the NMEA
# mode either: it was "not taken: 1 fix/s", a diag line and the
# GPS_NOT_TAKEN_SAY throttle started (the guard was in the PVT branch)
gl.reset()
_uart_rx[:] = []
gl.gps_setup()
sr_pre = (gl.st["setup_ok"], gl.st["checked"], gl.st["mode"])
_clock[0] += 2000
gl.handle(rv_nmea("GPTXT,01,01,02,u-blox AG - www.u-blox.com"))
sr_extra = gl.st["extra"]
for k in range(12):
    _clock[0] += 1000
    t = "1200%02d.00" % k
    gl.handle(rv_nmea("GPRMC,%s,A,4730.000,N,01900.000,E,0.1,,021126,,,A" % t))
    gl.handle(rv_nmea("GPGGA,%s,4730.000,N,01900.000,E,1,08,1.0,100.0,M,"
                      "40.0,M,," % t))
    gl.handle(rv_nmea("GPGST,%s,1.0,2.0,1.0,0.0,1.5,1.2,2.0" % t))
gl._setup_check(_clock[0])
check("SETUP-CHECK-RESTART: a restart within GPS_VERIFY is no 'not taken' "
      "in the NMEA mode",
      sr_pre == (True, False, "nmea") and gl.st["checked"] is False
      and gl.st["nt_said"] is None and gl.st["extra"] == sr_extra == 1,
      repr((sr_pre, gl.st["checked"], gl.st["nt_said"], gl.st["extra"])))
gl.reset()

# RATE-NAK: a CFG-RATE refused (NAK) is an answer, as NAV5's is: it was
# taken for silence (the blind burst, setup_ok False, set up again and
# again). The message frames go acknowledged, the receiver stays at its
# rate (1 fix/s is then no "not taken" verdict), a diag line says so


def rate_nak_run(nak):
    refused = _uart_nak_frames if isinstance(nak, bytes) else _uart_nak
    refused.add(nak)
    said = []
    saved_dw = gl._diag_write
    gl._diag_write = said.append
    try:
        n = verify_run(1, True)
        state = (gl.st["setup_ok"], gl.st["nav5_ok"], gl.st["checked"],
                 gl.st["extra"])
    finally:
        refused.discard(nak)
        gl._diag_write = saved_dw
    return n, state, [ln for ln in said if "rate" in ln]


rn1 = rate_nak_run((0x06, 0x08))           # every CFG-RATE refused
rn5 = rate_nak_run(gl._rate(1000 // gl.GPS_HZ))
check("RATE-NAK: CFG-RATE refused -> setup ok at 1 Hz, not set up again",
      rn1[:2] == (0, (True, True, True, 0)) and rn5[:2] == rn1[:2]
      and rn1[2] == ["gps rate %d ms refused" % 1000,
                     "gps rate %d ms refused" % (1000 // gl.GPS_HZ)]
      and rn5[2] == ["gps rate %d ms refused" % (1000 // gl.GPS_HZ)],
      repr((rn1, rn5)))

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
# SETUP-BACKOFF-CAP: ack_n grows for as long as the receiver keeps ignoring
# (a retry a minute: 43200 after a month), and GPS_RETRY << (n - 1) grew
# with it, a long int per setup (5 KB at 43200) on a ~15 KB free heap. A
# count no receiver reaches shows it here (CPython: too many digits)
cap_waits = []
for n_ in (2, 4, 5, 43200, 1 << 70):
    gl.reset()
    _uart_rx[:] = []
    gl.st["ack_n"] = n_
    try:
        gl.gps_setup()
        cap_waits.append(gl.st["setup_wait"])
    except (OverflowError, MemoryError) as e_:
        cap_waits.append(repr(e_))
check("SETUP-BACKOFF-CAP: the shift is capped, the wait GPS_IGNORED_MAX",
      cap_waits == [2 * gl.GPS_RETRY, 8 * gl.GPS_RETRY]
      + [gl.GPS_IGNORED_MAX] * 3, repr(cap_waits))

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

# DIAG-24H: the two diag files hold a day of status lines (~220 B a minute
# with ubx/hacc on NAV-PVT): the page's Event log aggregates all of it
check("DIAG-24H: DIAG_FILE + DIAG_OLD hold 24 h of 220 B status lines",
      2 * gl.DIAG_MAX >= 24 * 60 * 220 and gl.DIAG_MAX <= 256 * 1024,
      repr(gl.DIAG_MAX))


# DIAG-WEB: /diag?f=<0|1>&o=<n>: the raw bytes of DIAG_FILE (f=0) or
# DIAG_OLD (f=1) from byte n, HIST_REPLY at most a reply, an exact
# Content-Length and X-Diag-Size (the file's size: the page reloads both
# once DIAG_FILE is smaller than where it got to); streamed in WEB_CHUNK
# pieces, the UART read between them; a missing file, a bad f or o a 404,
# a foreign Host a 403
class _DConn(object):
    def __init__(self):
        self.sent, self.sends = b"", []

    def settimeout(self, t):
        pass

    def sendall(self, b):
        self.sends.append(len(b))
        self.sent += bytes(b)


def _dget(q, host=b""):
    c = _DConn()
    gl._web_reply(c, b"GET /diag?" + q + b" HTTP/1.1\r\n" + host + b"\r\n")
    head, _, body = c.sent.partition(b"\r\n\r\n")
    hs = head.split(b"\r\n")
    hd = {}
    for h in hs[1:]:
        k, _, v = h.partition(b": ")
        hd[k.decode()] = v.decode()
    return hs[0], hd, body, c.sends


diag_clear()
dw_old = b"".join((st_line(k) + "\n").encode() for k in range(120))
dw_cur = (b"wifi fail caf\xc3\xa9 status 201 after 12000 ms\n"
          + b"".join((st_line(k) + "\n").encode() for k in range(120, 230)))
with open(gl.DIAG_OLD, "wb") as f:
    f.write(dw_old)
with open(gl.DIAG_FILE, "wb") as f:
    f.write(dw_cur)
dw_uart = [0]
dw_real_uart = gl.read_uart


def _dw_uart():
    dw_uart[0] += 1


gl.read_uart = _dw_uart
try:
    dw_pages, dw_ok = {}, True
    for fn, want in ((1, dw_old), (0, dw_cur)):
        o, got = 0, b""
        for _ in range(50):
            st_, hd, body, sends = _dget(b"f=%d&o=%d" % (fn, o))
            dw_ok = (dw_ok and st_ == b"HTTP/1.0 200 OK"
                     and hd.get("Content-Length") == str(len(body))
                     and hd.get("X-Diag-Size") == str(len(want))
                     and len(body) <= gl.HIST_REPLY
                     and max(sends) <= gl.WEB_CHUNK)
            got += body
            o += len(body)
            if not body or o >= int(hd.get("X-Diag-Size", 0)):
                break
        dw_pages[fn] = (got == want, o)
    dw_past = _dget(b"f=0&o=999999")
    dw_mid = _dget(b"f=0&o=100")
    uart_n = dw_uart[0]
    dw_bad = [_dget(q)[0] for q in (b"f=2&o=0", b"f=x&o=0", b"f=0&o=-1",
                                    b"f=0&o=x", b"o=0", b"f=0&o=1x")]
    dw_host = _dget(b"f=0&o=0", b"Host: evil.example\r\n")[0]
    os.remove(gl.DIAG_OLD)
    dw_missing = _dget(b"f=1&o=0")[0]
finally:
    gl.read_uart = dw_real_uart
check("DIAG-WEB: /diag?f=1 and f=0 paged from o are the files byte for "
      "byte (a UTF-8 SSID too), HIST_REPLY a reply at most, exact "
      "Content-Length, X-Diag-Size the file's size, WEB_CHUNK sends, the "
      "UART read between them",
      dw_ok and dw_pages == {1: (True, len(dw_old)), 0: (True, len(dw_cur))}
      and len(dw_old) > gl.HIST_REPLY and uart_n > 0
      and dw_mid[2] == dw_cur[100:100 + gl.HIST_REPLY],
      repr((dw_ok, dw_pages, len(dw_old), uart_n)))
check("DIAG-WEB: an o past the end is a 200 without bytes and the size "
      "(the page sees a rotation by it)",
      dw_past[0] == b"HTTP/1.0 200 OK" and dw_past[2] == b""
      and dw_past[1].get("Content-Length") == "0"
      and dw_past[1].get("X-Diag-Size") == str(len(dw_cur)),
      repr(dw_past[:3]))
check("DIAG-WEB: a bad f or o, or a missing file, is a 404; a foreign Host "
      "a 403",
      dw_bad == [b"HTTP/1.0 404 Not Found"] * 6
      and dw_missing == b"HTTP/1.0 404 Not Found"
      and dw_host == b"HTTP/1.0 403 Forbidden",
      repr((dw_bad, dw_missing, dw_host)))
diag_clear()

# GPS-20: a cut-off sentence glued to a whole one keeps the whole one
gl.reset()
gl.handle(b"$GNGGA,120000.80,4728.0" + rv_nmea(
    "GNRMC,120001.00,A,4728.0000,N,01900.0000,E,0.1,,291026,,,A,V"))
check("GPS-20: the whole sentence after a cut-off one is parsed",
      gl.st["ok"] == 1 and gl.st["err"] == 0 and gl.ep["r"] is not None
      and gl.ep["date"] == "2026-10-29",
      "%d %d" % (gl.st["ok"], gl.st["err"]))

# HDOP-HOLD: a second with RMC only (its GGA lost) set hdop None, and an
# unknown HDOP passes _usable: the MAX_HDOP gate was off for that second.
# Now it keeps the last GGA HDOP and its stamp, for STALE ms (_fresh_hdop)
gl.reset()
hh_t0 = _clock[0]
gl.handle(rv_nmea("GPGGA,120000.00,4730.000,N,01900.000,E,1,08,9.0,100.0,M,"
                  "40.0,M,,"))
hh = []
for k in range(1, 4):
    _clock[0] = hh_t0 + 1000 * k
    gl.handle(rv_nmea("GPRMC,1200%02d.00,A,4730.000,N,01900.000,E,0.1,,"
                      "021126,,,A" % k))
    hh.append((gl.st["time"], gl.st["hdop"], gl._usable(gl.st["pos_at"])))
check("HDOP-HOLD: an RMC-only second keeps the last GGA HDOP for STALE ms",
      hh == [("12:00:00", 9.0, False), ("12:00:01", 9.0, False),
             ("12:00:02", None, True)]
      and gl.STALE == 2000, repr(hh))
gl.reset()

# NMEA-RANGE: a checksum-valid corrupt sentence (the XOR misses two flipped
# bits) put what float() takes in st: minutes 60+, lat 91, lon 181, "e"
# exponents (lat 5e32), nan / inf. Now such a sentence is bad (counted, none
# of it used); the edges and the plain forms are read as before
nr_bad = ("GPGGA,120001.00,4760.000,N,01900.000,E,1,08,1.0,100.0,M,,M,,",
          "GPGGA,120001.00,9100.000,N,01900.000,E,1,08,1.0,100.0,M,,M,,",
          "GPGGA,120001.00,4730.000,N,18100.000,E,1,08,1.0,100.0,M,,M,,",
          "GPGGA,120001.00,4730.0000e33,N,01900.000,E,1,08,1.0,100.0,M,,M,,",
          "GPGGA,120001.00,47e30.000,N,01900.000,E,1,08,1.0,100.0,M,,M,,",
          "GPGGA,120001.00,-4730.000,N,01900.000,E,1,08,1.0,100.0,M,,M,,",
          "GPGGA,120001.00,4730.000,X,01900.000,E,1,08,1.0,100.0,M,,M,,",
          "GPGGA,120001.00,4730.000,N,01900.000,N,1,08,1.0,100.0,M,,M,,",
          "GPGGA,120001.00,4730.000,N,01900.000,E,1,08,1.0,001.0e9,M,,M,,",
          "GPGGA,120001.00,4730.000,N,01900.000,E,1,08,nan,100.0,M,,M,,",
          "GPGGA,120001.00,4730.000,N,01900.000,E,1,08,1.0,inf,M,,M,,",
          "GPRMC,120001.00,A,4730.000,N,01900.000,E,inf,,021126,,,A",
          "GPRMC,120001.00,A,4730.000,N,01900.000,E,1e3,,021126,,,A",
          "GPRMC,120001.00,A,4730.000,N,1900.0.00,E,0.1,,021126,,,A",
          "GPGST,120001.00,1.0,2.0,1.0,0.0,nan,1.2,2.0")
nr_ok = (("GPGGA,120001.00,8959.9999,S,17959.9999,W,1,08,1.0,-12.5,M,,M,,",
          (-89.99999833, -179.99999833)),
         ("GPGGA,120001.00,9000.0000,N,18000.0000,E,1,08,1.0,1.0,M,,M,,",
          (90.0, 180.0)),
         ("GPRMC,120001.00,A,0000.000,N,00000.000,E,0.1,,021126,,,A",
          (0.0, 0.0)),
         ("GPRMC,120001.00,A,4730,N,1900.5,E,0.1,,021126,,,A",
          (47.5, 19.00833333)))
nr_got = []
for body in nr_bad:
    gl.reset()
    gl.handle(rv_nmea(body))
    gl.handle(rv_nmea("GPRMC,120002.00,V,,,,,,,021126,,,N"))
    nr_got.append((body, gl.st["err"], gl.st["valid"], gl.st["lat"],
                   gl.st["alt"], gl.st["hdop"], gl.st["speed"],
                   gl.st["std"], gl.ep["spd"]))
nr_fail = [g for g in nr_got
           if g[1:] != (1, False, None, None, None, None, None, None)]
nr_okgot = []
for body, want in nr_ok:
    gl.reset()
    gl.handle(rv_nmea(body))
    gl.handle(rv_nmea("GPRMC,120002.00,V,,,,,,,021126,,,N"))
    nr_okgot.append((gl.st["err"], gl.st["valid"],
                     gl.st["lat"] is not None
                     and round(gl.st["lat"], 6) == round(want[0], 6)
                     and round(gl.st["lon"], 6) == round(want[1], 6)))
gl.reset()
check("NMEA-RANGE: minutes 60+, |lat| > 90, |lon| > 180, exponents, signs, "
      "bad hemispheres, nan / inf -> the sentence is bad, nothing kept",
      not nr_fail, repr(nr_fail[:3]))
check("NMEA-RANGE: 90 / 180 degrees, 0, no fraction still read",
      nr_okgot == [(0, True, True)] * len(nr_ok), repr(nr_okgot))

# GPS-21: uptime sums tick differences across a ticks_ms wrap
TPW = 1 << 30
saved_t = (time.ticks_ms, time.ticks_diff)
time.ticks_diff = lambda a, b: ((a - b + TPW // 2) & (TPW - 1)) - TPW // 2
try:
    gl._up[:] = [0, TPW - 1000, 0]
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

# NMEA-INPLACE: handle() reads a sentence in the buffer (read_uart's bounds,
# no copy) and decodes only the types it reads; it counts and parses as the
# copying one did (here as it was, the reference) on random sentences: bad
# checksums, odd hex, junk and blanks around, unknown types, other address
# lengths, bytes over 0x7F, the PVT mode's "turned off" count


def old_handle(line):
    line = line.strip()
    i = line.rfind(b"$")
    if i > 0:
        line = line[i:]
    if not line:
        return
    star = line.rfind(b"*")
    ok = line.startswith(b"$") and star >= 0
    if ok:
        c = 0
        for b in memoryview(line)[1:star]:
            c ^= b
        t = line[star + 1:]               # exactly two hex digits
        ok = (len(t) == 2 and all(x in b"0123456789abcdefABCDEF" for x in t)
              and c == int(t, 16))
    if not ok:
        gl.st["err"] += 1
        return
    gl.st["good"] = time.ticks_ms()
    try:
        f = line[1:line.rfind(b"*")].decode().split(",")
        fn = gl.HANDLERS.get(f[0][2:])
        if fn:
            fn(f[0][:2], f)
            if gl.st.get("mode") == "pvt" and f[0][2:] in gl._PVT_OFF:
                gl._extra(f[0][:2], f)
        gl.st["ok"] += 1
    except (ValueError, IndexError, UnicodeError):
        gl.st["err"] += 1


ip_rnd = random.Random(29)
ip_bodies = ("GPRMC,120000.00,A,4730.000,N,01900.000,E,0.1,90.0,171026,,,A",
             "GNGGA,120000.20,4730.000,N,01900.000,E,1,08,1.0,100.0,M,,M,,",
             "GPGSA,A,3,01,02,,,,,,,,,,,1.5,0.9,1.2", "GLGSV,3,1,12,01,40,083,46",
             "GPGST,120000.00,20,,,,3.0,2.5,6.0", "GPVTG,,T,,M,0.0,N,0.0,K,A",
             "GPTXT,01,01,02,u-blox ag", "GPZDA,120000.00,17,10,2026,00,00",
             "PUBX,00,120000", "GPRMCX,1,2", "GPRM", "GPRMC", "GPGGA,1",
             "GP\xe9MC,1,A", "GPRMC,12\xe90000.00,A,,,,,,,171026,,,N",
             "\xe9GRMC,120000.00,A,4730.000,N,01900.000,E,0.1,,171026,,,A",
             "\xe9GGSV,3,1,12,01,40,083,46")
ip_keys = ("ok", "err", "good", "valid", "lat", "time", "date", "quality", "used",
           "fixtype", "view", "extra", "speed", "seq")
ip_bad = []
for n_ in range(1500):
    body = ip_rnd.choice(ip_bodies).encode()
    if ip_rnd.random() < 0.3:
        k = ip_rnd.randrange(len(body))
        body = body[:k] + bytes((ip_rnd.randrange(256),)) + body[k + 1:]
    c = 0
    for ch in body:
        c ^= ch
    ck = ip_rnd.choice(("%02X" % c, "%02x" % c, "%02X" % (c ^ 1), "%X" % c,
                        "", " %X" % c, "%02X\r" % c, "%02Xzz" % c, "+%X" % c))
    line = b"$" + body + b"*" + ck.encode()
    if ip_rnd.random() < 0.2:
        line = ip_rnd.choice((b"  ", b"\xb5b\x01", b"$GPGG", b"\t")) + line
    if ip_rnd.random() < 0.2:
        line += ip_rnd.choice((b" ", b"\r", b"*", b"$"))
    pad = bytearray(b"xx\n" + line + b"\r\n12")
    mode = "pvt" if n_ % 3 == 0 else "nmea"
    got = []
    for run in range(2):
        gl.reset()
        gl.st.update(mode=mode, setup_at=_clock[0] - 5000)
        if run:
            gl.handle(pad, 3, 3 + len(line))
        else:
            old_handle(line)
        gl.handle(rv_nmea("GPRMC,120001.00,V,,,,,,,171026,,,N"))
        got.append([gl.st[k_] for k_ in ip_keys])
    if got[0] != got[1]:
        ip_bad.append((line, got))
check("NMEA-INPLACE: in place it counts and parses as the copying handle()",
      not ip_bad, "%d differ, first %r" % (len(ip_bad), ip_bad[:1]))

# NMEA-CK-STRICT: the checksum is exactly two hex digits after the "*" and
# nothing else: the int(..., 16) fallback took "*+F", "* 5", "*5" and
# "*5Azz" (a checksum of 5 that a corrupt line matches more easily)
cs_c = 0                                  # (field 10, the variation, not
for ch in b"GPRMC,120000.00,A,4730.000,N,01900.000,E,0.1,,021126,,,A":
    cs_c ^= ch                            # read: a checksum under 16)
cs_x = [x for x in range(0x30, 0x7B) if chr(x).isalnum() and cs_c ^ x < 16]
cs_body = "GPRMC,120000.00,A,4730.000,N,01900.000,E,0.1,,021126,%s,,A" % (
    chr(cs_x[0]))
cs_c ^= cs_x[0]
cs_got = []
for ck_ in ("+%X", " %X", "%X", "%02Xzz", "%02X*", "-%X", "%02X", "%02x"):
    gl.reset()
    gl.handle(("$%s*%s\r\n" % (cs_body, ck_ % cs_c)).encode())
    cs_got.append((gl.st["ok"], gl.st["err"]))
gl.reset()
check("NMEA-CK-STRICT: only two hex digits after '*' (not '+F', ' 5', '5')",
      cs_c < 16 and cs_got == [(0, 1)] * 6 + [(1, 0)] * 2,
      repr((cs_c, cs_got)))


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

# RXBUF-COMMIT: _rx_n was set before the loop, the drop of what was read
# only after it: an exception in a handler (MemoryError while publishing)
# left the buffer uncommitted and the next read_uart handled the same
# sentences again (queued seconds 10, 11, 10, 11, 12). Now a handler's
# error counts the line bad and read_uart always commits what it read
rc_list = [rv_nmea("GPGGA,%02d0000.00,4730.0000,N,01900.0000,E,1,08,1.0,"
                   "100.0,M,40.0,M,," % (10 + i)) + b"\r\n" for i in range(4)]
rc_lines = b"".join(rc_list)
rc_got = []
for rc_err in (MemoryError, RuntimeError):
    gl.reset()
    _uart_rx[:] = []
    gl._rx_n = gl._rx_s = 0
    rc_calls = [0]
    rc_gga = gl.HANDLERS["GGA"]

    def rc_boom(t, f, err=rc_err):
        rc_calls[0] += 1
        if rc_calls[0] == 3:
            raise err("in a handler")
        rc_gga(t, f)
    gl.HANDLERS["GGA"] = rc_boom
    _uart_rx.extend(rc_lines)
    try:
        try:
            gl.read_uart()
            rc_raised = None
        except Exception as e:
            rc_raised = type(e).__name__
        gl.read_uart()
    finally:
        gl.HANDLERS["GGA"] = rc_gga
    rc_got.append((rc_raised, rc_calls[0], [s[9] for s in gl.st["q"]],
                   gl.st["err"], gl._rx_n))
rc_ubx = gl._ubx_frame
gl.reset()
gl._rx_n = gl._rx_s = 0
_uart_rx[:] = []
_uart_rx.extend(gl.MON_VER + rc_list[0] + rc_list[1])


def rc_ubx_boom(fr):
    raise MemoryError("in _ubx_frame")


gl._ubx_frame = rc_ubx_boom
try:
    try:
        gl.read_uart()
        rc_u = None
    except Exception as e:
        rc_u = type(e).__name__
finally:
    gl._ubx_frame = rc_ubx
rc_u = (rc_u, gl.st["err"], gl._rx_n, gl.st["ok"])
gl.reset()
gl._rx_n = gl._rx_s = 0
_uart_rx[:] = []
check("RXBUF-COMMIT: a handler's error counts the line, nothing handled twice",
      rc_got == [(None, 4, ["10:00:00", "11:00:00"], 1, 0)] * 2
      and rc_u == (None, 1, 0, 2), repr((rc_got, rc_u)))


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
check("PVT-PARSE: latest epoch's speed, sats, fix type",
      gl.st["used"] == 9 and gl.st["fixtype"] == 3 and gl.st["quality"] == 1
      and abs(gl.st["speed"] - 7.2) < 1e-9 and gl.status() == "3D fix",
      repr((gl.st["used"], gl.st["speed"])))
# (PDOP, course, gSpeed, sAcc, vAcc, tAcc, RMC status: stored every frame /
# sentence, read by nothing: not in st)
gl.handle(rv_nmea("GPRMC,120001.00,A,4730.000,N,01900.000,E,0.1,12.5,"
                  "311026,,,A"))
gl.handle(rv_nmea("GPGSA,A,3,01,02,,,,,,,,,,,1.5,0.9,1.2"))
pp_dead = [k for k in ("pdop", "course", "head", "gspeed", "sacc", "vacc",
                       "tacc", "status") if k in gl.st]
check("PVT-PARSE: no store nothing reads (PDOP, course, accuracies, status)",
      pp_dead == [] and gl.st["fixtype"] == 3, repr(pp_dead))


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
# PVT-THIN: a POST blocking 40 s (a weak hotspot while driving) left 40
# seconds published with no log_tick: the queue kept the newest
# SECONDS_Q, so the first 32 seconds were a gap in the track with no
# segment break. Full, it drops the second whose neighbours are closest:
# the same SECONDS_Q seconds spread over the whole block, start and end
# kept, in order, the thinned ones counted
th_td = tempfile.mkdtemp(prefix="pvtthin-")
saved_dir = gl.LOG_DIR
try:
    gl.LOG_DIR = th_td
    gl.reset()
    gl.log_reset()
    _uart_rx[:] = []
    for s_ in range(41):
        for k in range(5):
            _uart_rx.extend(pvt_frame(t_at(s_), 475000000 + 2000 * s_,
                                      190000000, nano=k * 200000000))
    gl.read_uart()
    th_n = gl.st.get("q_thin")
    gl.log_tick()
    th_lines = list(gl.log["batch"])
finally:
    gl.LOG_DIR = saved_dir
    shutil.rmtree(th_td, ignore_errors=True)
th_t = [l.rstrip().rsplit(",", 1)[1] for l in th_lines]
check("PVT-THIN: 40 s published without log_tick keep SECONDS_Q seconds "
      "spread over all of them, not the newest only",
      len(th_lines) == gl.SECONDS_Q and th_t[0] == "20261031T120000Z"
      and th_t[-1] == "20261031T120039Z" and th_t == sorted(th_t)
      and th_n == 40 - gl.SECONDS_Q, repr((th_n, th_t)))
th_rows = dict(gl._web_rows())
check("PVT-THIN: status.json counts thinned seconds, lost points and track "
      "write errors", th_rows.get("q_thin") == 40 - gl.SECONDS_Q
      and "track_drop" in th_rows and "write_errs" in th_rows,
      repr({k: th_rows.get(k) for k in ("q_thin", "track_drop",
                                        "write_errs")}))

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

# HIST: the day history the status page draws - each point once it is
# uploaded, appended to HIST_DIR/<UTC day>.trb (the track file is deleted
# then), the newest HIST_DAYS days kept, served by /hist?d=&o= as its text
# lines. A day begun as text (.trk, older firmware) stays text.
import tempfile as _tf
_hist_saved = (gl.HIST_DIR, gl.LOG_DIR, gl.HIST_REPLY)
_hd = _tf.mkdtemp(prefix="hist-")
gl.HIST_DIR = os.path.join(_hd, "hist")
gl.LOG_DIR = _hd


def _hl(day, hms, lat=47.185):
    return "%.6f,18.427300,100.0,2,1.4,%sT%sZ\n" % (lat, day, hms)


def _hraw(day, ext=".trb"):
    try:
        with builtins.open(os.path.join(gl.HIST_DIR, day + ext), "rb") as f:
            return f.read()
    except OSError:
        return None


def _hpts(data):
    """The point lines a day file's bytes hold: a .trb decoded by trk2gpx
    (apart from the logger's own decoder), a .trk as it is."""
    if data is None:
        return None
    if not data.startswith(b"#TRKB\n"):
        return data.decode()
    return "".join(x for x in trk2gpx.trb_lines(data)
                   if x[:1] != "#" and x != "-\n")


def _hread(day):
    return _hpts(_hraw(day))


def _hkb(ls):
    """The .trb records a flush writes for the text lines ls."""
    return bytes(gl._kb_encode(ls, len(ls), None))


def _drive(day, n, t0=36000, fmt=2, seed=1):
    """n lines of a drive on the UTC day (YYYYMMDD) from t0 s into it, as
    _sample_line writes them: 1 Hz (now and then a second skipped), ~45
    km/h with turns and stops, ele/acc/speed changing as a receiver's do.
    fmt 2: lat,lon,ele,acc,speed; 1: lat,lon,ele,hdop."""
    import random
    r = random.Random(seed)
    t = (gl._days(int(day[:4]), int(day[4:6]), int(day[6:]))
         - gl.EPOCH_DAYS) * 86400 + t0
    la, lo, el, hd, v, acc = 471853560, 184269340, 1203, 0.6, 12.0, 21
    out = []
    for i in range(n):
        v = min(max(v + r.uniform(-1.2, 1.2), 0.0), 25.0)
        if i % 300 > 280:
            v = max(v - 3, 0.0)          # (a stop at a light)
        hd += r.uniform(-0.08, 0.08)
        la += int(v * math.cos(hd) * 89.8)
        lo += int(v * math.sin(hd) * 132.1)
        el += r.choice((-1, 0, 0, 0, 1))
        if r.random() < 0.2:
            acc = min(max(acc + r.choice((-1, 1)), 9), 60)
        s = gl._stamp_s(t)[1]
        if fmt == 2:
            out.append("%s,%s,%.1f,%.1f,%.1f,%s\n" % (
                gl._i7(la), gl._i7(lo), el / 10, acc / 10, v * 3.6, s))
        else:
            out.append("%s,%s,%.1f,%.1f,%s\n" % (
                gl._i7(la), gl._i7(lo), el / 10, acc / 10, s))
        t += 2 if r.random() < 0.03 else 1
    return out


class _HConn:
    def __init__(self):
        self.sent = b""

    def settimeout(self, t):
        pass

    def sendall(self, b):
        self.sent += bytes(b)


def _hget(q):
    """-> (status line, {header: value}, body) of GET /hist?q."""
    c = _HConn()
    gl._web_reply(c, b"GET /hist?" + q + b" HTTP/1.1\r\n\r\n")
    head, _, body = c.sent.partition(b"\r\n\r\n")
    hs = head.split(b"\r\n")
    hd = {}
    for h in hs[1:]:
        k, _, v = h.partition(b": ")
        hd[k.decode()] = v.decode()
    return hs[0], hd, body


def _hpage(day, o=0, most=10000):
    """Page through a day as the page does: from o, then from each reply's
    X-Hist-Next while it is short of X-Hist-End -> (the text, the replies:
    (o, next, end, body length, Content-Length))."""
    text, reps = b"", []
    for _ in range(most):
        st_, hd, body = _hget(b"d=%s&o=%d" % (day.encode(), o))
        if st_ != b"HTTP/1.0 200 OK" or "X-Hist-Next" not in hd:
            reps.append((o, None, None, len(body), None))
            break
        nx, end = int(hd["X-Hist-Next"]), int(hd["X-Hist-End"])
        reps.append((o, nx, end, len(body), int(hd["Content-Length"])))
        text += body
        if nx >= end or nx <= o:
            break
        o = nx
    return text.decode("latin-1"), reps


def _hok(line):
    """A line the page draws (histParse): 5+ fields, a 16-char time last,
    numbers first."""
    f = line.rstrip("\n").split(",")
    try:
        return len(f) >= 5 and len(f[-1]) == 16 and f[0] != "" and (
            float(f[0]) == float(f[0]) and float(f[1]) == float(f[1]))
    except ValueError:
        return False


try:
    lines = [_hl("20261030", "235958"), _hl("20261030", "235959"),
             _hl("20261031", "000000"), _hl("20261031", "000001")]
    gl._hist_add(lines + ["not written\n"], 4)
    gl._hist_add([_hl("20261031", "000002")], 1)
    h30, h31 = _hread("20261030"), _hread("20261031")
    check("HIST: lines go to their own UTC day's .trb, appended, only "
          "lines[:k]",
          h30 == "".join(lines[:2])
          and h31 == "".join(lines[2:]) + _hl("20261031", "000002")
          and _hraw("20261031", ".trk") is None, repr((h30, h31)))
    check("HIST: a day file is a .trb track: the header, each append the "
          "records a flush writes (a keyframe first: any append start is a "
          "decode start), one footer at the end (overwritten by the next)",
          _hraw("20261031") == b"#TRKB\n" + _hkb(lines[2:4])
          + _hkb([_hl("20261031", "000002")]) + b"#END\n",
          repr(_hraw("20261031")))
    for d_ in range(1, 10):               # 9 old (text) days, then a new one
        with open(os.path.join(gl.HIST_DIR, "202610%02d.trk" % d_), "w") as f:
            f.write(_hl("202610%02d" % d_, "120000"))
    gl._hist_add([_hl("20261101", "080000")], 1)
    kept = sorted(os.listdir(gl.HIST_DIR))
    check("HIST: a new day's file drops the days beyond HIST_DAYS (.trk "
          "and .trb alike)",
          len(kept) == gl.HIST_DAYS and kept[-1] == "20261101.trb"
          and kept[0] == "20261006.trk" and "20261030.trb" in kept,
          repr(kept))
    free_saved = gl.free
    gl.free = lambda: gl.MIN_FREE
    try:
        gl._hist_add([_hl("20261101", "080001")], 1)
    finally:
        gl.free = free_saved
    check("HIST: nothing written while the flash is nearly full",
          _hread("20261101") == _hl("20261101", "080000"))
    # flush: a .trb's points are on the flash once, in the track file; the
    # history gets them when they are up (they were written twice, and a
    # day offline filled the flash twice as fast). A .trk (TRK_BIN off, or
    # from older firmware, already in the history) goes at flush as before
    for ext in (".trk", gl.TRB):
        tb_ = ext == gl.TRB
        online(_hd)
        gl.LOG_DIR = _hd
        h0 = _hread("20261101")
        gl.log["file"] = os.path.join(_hd, "20261101_090000" + ext)
        gl.log["fmt"] = 2
        fl = [_hl("20261101", "090000"), _hl("20261101", "090001", 47.186)]
        gl.log["batch"] = list(fl)
        gl.flush()
        check("HIST: flush writes a .trb, not the day history; a .trk and "
              "the history (%s)" % ext,
              _hread("20261101") == (h0 if tb_ else h0 + "".join(fl))
              and gl.log["batch"] == [] and gl.log["points"] == 2,
              repr(_hread("20261101")))
        h0 = h0 if tb_ else h0 + "".join(fl)
        gl.log["file"] = None            # (closed: pending)
        _post_code[0] = 500
        pump()
        check("HIST: a failed upload adds nothing (%s)" % ext,
              _hread("20261101") == h0, repr(_hread("20261101")))
        _post_code[0] = 201
        pump()
        check("HIST: a .trb's uploaded points go to the day history, once, "
              "as its records; a .trk's not again (%s)" % ext,
              _hread("20261101") == h0 + ("".join(fl) if tb_ else "")
              and not os.path.exists(os.path.join(_hd, "20261101_090000"
                                                  + ext))
              and _hraw("20261101").endswith(_hkb(fl) + b"#END\n"),
              repr(_hraw("20261101")))
        pump()
        check("HIST: ... and not again (%s)" % ext,
              _hread("20261101") == h0 + ("".join(fl) if tb_ else ""))
        os.remove(os.path.join(gl.HIST_DIR, "20261101.trb"))
        gl._hist_add([_hl("20261101", "080000")], 1)
    gl.log_reset()

    # an uploaded batch across midnight: each day's file gets its part,
    # each part a keyframe first
    mb = os.path.join(_hd, "mid.trb")
    ml = _drive("20261102", 6, t0=86397)
    with builtins.open(mb, "wb") as f:
        f.write(b"#TRKB\n" + _hkb(ml) + b"#END\n")
    gl._hist_batch(mb, 0, os.path.getsize(mb), None)
    os.remove(mb)
    ma = [x for x in ml if x[-17:-9] == "20261102"]
    check("HIST: a batch across midnight goes to both days' files, each "
          "part a keyframe first",
          0 < len(ma) < len(ml)
          and _hraw("20261102") == b"#TRKB\n" + _hkb(ma) + b"#END\n"
          and _hraw("20261103") == b"#TRKB\n" + _hkb(ml[len(ma):])
          + b"#END\n",
          repr((_hraw("20261102"), _hraw("20261103"))))

    full = _hread("20261101").encode()
    a = _hget(b"d=20261101&o=0")
    end = _hget(b"d=20261101&o=99999")
    bad = [_hget(q)[0] for q in (b"d=2026110&o=0", b"d=../../x&o=0",
                                 b"d=20261101&o=-1", b"d=20261101&o=x",
                                 b"d=20250101&o=0")]
    rw = _hraw("20261101")
    check("HIST-WEB: /hist?d=&o= sends the day's .trb as its text lines "
          "(Content-Length), X-Hist-Next where to ask from next, X-Hist-End "
          "where its records end (the footer's start); an o past it: none",
          a[0] == b"HTTP/1.0 200 OK" and a[2] == full
          and a[1].get("Content-Length") == str(len(full))
          and a[1].get("X-Hist-Next") == a[1].get("X-Hist-End")
          == str(len(rw) - 5) and end[0] == b"HTTP/1.0 200 OK"
          and end[2] == b"" and end[1].get("X-Hist-Next") == str(len(rw) - 5),
          repr((a, end)))
    check("HIST-WEB: a bad day, offset or a missing day is a 404",
          bad == [b"HTTP/1.0 404 Not Found"] * 5, repr(bad))

    # a realistic day: 1 Hz drives uploaded in batches of UPLOAD_BATCH, a
    # TRK1 track among them, and lines the encoder keeps as keyframes (no
    # number as it prints, more than 9 digits): the replies together are
    # exactly the text history, each line once
    day = "20261104"
    dl = (_drive(day, 1500, t0=25200) + _drive(day, 130, t0=40000, fmt=1)
          + ["47.1850000,18.4273,100.00,2,1.4,20261104T120000Z\n",
             "47.1234567891,18.4,1,2,3,20261104T120001Z\n",
             "-0.000001,-0.000001,,1.2,20261104T120002Z\n"]
          + _drive(day, 900, t0=61200, seed=2))
    for i in range(0, len(dl), gl.UPLOAD_BATCH):
        gl._hist_add(dl[i:i + gl.UPLOAD_BATCH],
                     len(dl[i:i + gl.UPLOAD_BATCH]))
    raw = _hraw(day)
    txt, reps = _hpage(day)
    gl.HIST_REPLY = 2000
    try:
        txt2, reps2 = _hpage(day)
    finally:
        gl.HIST_REPLY = _hist_saved[2]
    big = 65 * max(len(x) for x in dl)
    check("HIST-WEB: a day read reply by reply is exactly its text lines, "
          "each once (16 KB replies and 2 KB ones)",
          txt == txt2 == "".join(dl) == _hread(day) and len(reps) > 5
          and len(reps2) > 40, repr((len(txt), len(txt2), len(reps))))
    check("HIST-WEB: each reply stops before a keyframe once HIST_REPLY "
          "bytes are out (the last one at X-Hist-End), its Content-Length "
          "what it sent",
          all(n_ == cl and (raw[nx:nx + 1] == b"=" or nx == e_)
              and (nx == e_ or HIST_REPLY_ <= n_ <= HIST_REPLY_ + big)
              for HIST_REPLY_, rr in ((16384, reps), (2000, reps2))
              for o_, nx, e_, n_, cl in rr)
          and reps[-1][1] == reps[-1][2] == len(raw) - 5,
          repr((reps[:3], reps2[:3])))
    # the page asks from where it got to while a batch is added; then a
    # minute later only for what is new (from X-Hist-End: the new append's
    # keyframe overwrote the footer there)
    day = "20261105"
    pl = _drive(day, 400, t0=30000)
    gl._hist_add(pl[:200], 200)
    gl.HIST_REPLY = 2000
    try:
        st1, h1, b1 = _hget(b"d=%s&o=0" % day.encode())
        gl._hist_add(pl[200:300], 100)
        t2, r2 = _hpage(day, int(h1["X-Hist-Next"]))
        e2 = r2[-1][2]
        st3, h3, b3 = _hget(b"d=%s&o=%d" % (day.encode(), e2))
        gl._hist_add(pl[300:], 100)
        t4, r4 = _hpage(day, e2)
    finally:
        gl.HIST_REPLY = _hist_saved[2]
    check("HIST-WEB: an append while the page reads the day: no line lost "
          "or twice; at the end only what is new",
          b1.decode() + t2 + b3.decode() + t4 == "".join(pl)
          and int(h1["X-Hist-Next"]) < int(h1["X-Hist-End"])
          and b3 == b"" and h3["X-Hist-Next"] == str(e2)
          and t4 == "".join(pl[300:]),
          repr((len(b1), len(t2), len(b3), len(t4))))

    # a cut last append (ENOSPC mid-write, power gone before the footer):
    # the next append starts after HIST_SEP (it ends any record cut there:
    # its first varint is no keyframe's "=") and is read whole; the cut
    # record and the half keyframe are no points, none made up
    day = "20261106"
    ca = _drive(day, 130, t0=30000)
    cb = _drive(day, 20, t0=40000, seed=3)
    for i in range(0, 130, 50):
        gl._hist_add(ca[i:i + 50], len(ca[i:i + 50]))
    craw = _hraw(day)
    last = len(b"#TRKB\n" + _hkb(ca[:50]) + _hkb(ca[50:100]))
    cpath = os.path.join(gl.HIST_DIR, day + ".trb")
    bad_cuts = []
    for cut in range(last, len(craw)):
        with builtins.open(cpath, "wb") as f:
            f.write(craw[:cut])
        pre = _hpts(craw[:cut])          # (trk2gpx: what the cut part holds)
        k = pre.count("\n")
        t0_, r0_ = _hpage(day)           # (the page, before the next append)
        gl._hist_add(cb, len(cb))
        t1_, r1_ = _hpage(day)
        tn_, _ = _hpage(day, r0_[-1][1])
        t1v = "".join(x for x in t1_.splitlines(True) if _hok(x))
        tnv = "".join(x for x in tn_.splitlines(True) if _hok(x))
        gl._hist_add(ca[:2], 2)          # (the next append: no HIST_SEP)
        ok = (pre == "".join(ca[:k]) and k >= 100
              and "".join(x for x in t0_.splitlines(True) if _hok(x)) == pre
              and t1v == pre + "".join(cb) and tnv == "".join(cb)
              and _hraw(day).endswith(_hkb(cb) + _hkb(ca[:2]) + b"#END\n")
              and _hraw(day).count(gl.HIST_SEP) == (cut < len(craw)))
        if not ok:
            bad_cuts.append(cut)
    with builtins.open(cpath, "wb") as f:
        f.write(craw)
    check("HIST-CUT: an append cut at any byte of the last one: the next "
          "append and every point before the cut are read, nothing made up "
          "(%d cuts)" % (len(craw) - last), not bad_cuts, repr(bad_cuts[:8]))

    # an erased block (0xFF) in a day: no keyframe for 40 KB. A reply still
    # stops after HIST_REPLY / 4 bytes of the file (anywhere while no
    # keyframe holds: nothing decodes there) and the lines after it are read
    day = "20261107"
    el_ = _drive(day, 120, t0=30000)
    gl._hist_add(el_[:60], 60)
    with builtins.open(os.path.join(gl.HIST_DIR, day + ".trb"), "r+b") as f:
        f.seek(-5, 2)
        f.write(b"\xff" * 40000)         # (no footer: the next append's
    gl._hist_add(el_[60:], 60)           # HIST_SEP ends the block)
    te, re_ = _hpage(day)
    check("HIST-WEB: an erased block of 40 KB in a day is read in "
          "HIST_REPLY / 4 steps, the lines around it whole",
          te == "".join(el_) and len(re_) >= 10
          and all(nx - o_ <= gl.HIST_REPLY // 4 + 64
                  for o_, nx, e_, n_, cl in re_),
          repr(re_))

    # a long day: one reply reads about HIST_REPLY of text whatever the
    # day's length, feeding the WDT as it goes (a 6 h drive: 21600 points)
    day = "20261108"
    ll = _drive(day, 21600, t0=21600, seed=4)
    for i in range(0, len(ll), 600):
        gl._hist_add(ll[i:i + 600], 600)
    dog = WDT(timeout=gl.WDT_TIMEOUT)
    gl._wdt = dog
    try:
        import tracemalloc
        tracemalloc.start()
        rl = _hget(b"d=%s&o=0" % day.encode())
        peak = tracemalloc.get_traced_memory()[1]
        tracemalloc.stop()
        fed = dog.fed
        tl_, rr_ = _hpage(day)
    finally:
        gl._wdt = None
    check("HIST-WEB: a 6 h day: a reply is ~HIST_REPLY (the WDT fed as it "
          "decodes, no list of the day in RAM), every point once over all",
          16384 <= len(rl[2]) <= 16384 + big and fed >= 2 * 330 // 64
          and peak < 64 * 1024 and tl_ == "".join(ll)
          and len(_hraw(day)) < 9 * len(ll),
          repr((len(rl[2]), fed, peak, len(_hraw(day)) / len(ll))))
    # the flash it takes: an uploaded .trb's batches (UPLOAD_BATCH points)
    # are the appends, a keyframe each
    sz_t = sum(len(x) for x in ll)
    sz_b = len(_hraw(day))
    sz_u = 11 + sum(len(_hkb(ll[i:i + gl.UPLOAD_BATCH]))
                    for i in range(0, len(ll), gl.UPLOAD_BATCH))
    print("HIST-SIZE: a 1 Hz drive's day history: %.1f B a point as text, "
          "%.1f as .trb in %d-point appends, %.1f in 600-point ones" % (
              sz_t / len(ll), sz_u / len(ll), gl.UPLOAD_BATCH,
              sz_b / len(ll)))
    check("HIST-SIZE: a 1 Hz drive's day history is under a sixth of its "
          "text as .trb (uploaded in UPLOAD_BATCH appends)",
          sz_u * 6 < sz_t and sz_b < sz_u, repr((sz_t, sz_u, sz_b)))

    # older firmware's text days: listed with the .trb ones, served as
    # they are (no X-Hist-Next: the page counts bytes), and an append to
    # such a day stays text (no second file for a day, no conversion)
    gl._hd[1] = None
    tday = "20261103"
    with open(os.path.join(gl.HIST_DIR, "20261009.trk"), "w") as f:
        f.write(_hl("20261009", "120000"))
    ot = _hraw("20261009", ".trk")
    gl._hist_add([_hl("20261009", "120001")], 1)
    ta = _hget(b"d=20261009&o=0")
    tb2 = _hget(b"d=20261009&o=10")
    days_ = gl._hist_days()
    check("HIST-MIGRATE: a .trk day (older firmware) is listed with the "
          ".trb days, served as text from byte o without X-Hist-Next, and "
          "appended to as text",
          ta[2] == ot + _hl("20261009", "120001").encode()
          and "X-Hist-Next" not in ta[1] and tb2[2] == ta[2][10:]
          and ta[1].get("Content-Length") == str(len(ta[2]))
          and _hraw("20261009") is None and _hraw(tday) is not None
          and "20261009" in days_ and tday in days_
          and days_ == sorted(set(days_)), repr((ta, days_)))
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
check("PVT-SETUP: M10 - one CFG-VALSET (RAM): NAV-PVT on, GGA/RMC/GSA/GST/"
      "GSV/GLL/VTG off, between the 1 Hz and 5 Hz rate frames",
      gl.st["mode"] == "pvt" and gl.st["setup_ok"] is True
      and vs_frame in fr10 and not nmea_set(fr10)
      and fr10[0] == gl._rate(1000) and fr10[-1] == gl._rate(200)
      and fr10.index(vs_frame) == 2 and len(fr10) == 4,
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
      gl.st["mode"] == "nmea" and fr_off[2:-1] == want_nmea + [pvt_stop])
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

# PVT-NAK-STOP: an M8 that refused a frame of its NAV-PVT setup (or stopped
# answering) got the frames after it anyway, blind: the NMEA-off ones left
# it silent, no NMEA and no NAV-PVT, until the next setup (a running
# backoff: minutes). Now the setup stops there, and a NAK makes the NMEA
# setup due within GPS_RETRY


def m8_stop(nak=None, silent=None):
    saved = gl.GPS_PVT
    gl.GPS_PVT = True
    gl.reset()
    _uart_rx[:] = []
    gl._ubx_frame(M8_VER)
    gl.st["setup_wait"] = gl.GPS_RETRY_MAX       # a backoff running
    _uart_nak_frames.add(nak)
    _uart_silent.add(silent)
    try:
        _uart_tx[:] = []
        gl.gps_setup()
        sent = [f for f in _uart_tx if f[2:3] == b"\x06"]
        state = (gl.st["pvt_bad"], gl.st["setup_ok"], gl.st["setup_wait"])
    finally:
        _uart_nak_frames.discard(nak)
        _uart_silent.discard(silent)
        gl.GPS_PVT = saved
    _uart_tx[:] = []
    _clock[0] += gl.GPS_RETRY + 1000
    gl.gps_check(_clock[0])
    return sent, state, list(_uart_tx)


s_on, st_on, again_on = m8_stop(nak=want8[0])
s_gsa, st_gsa, _ = m8_stop(nak=want8[3])
s_sil, st_sil, _ = m8_stop(silent=(0x06, 0x01))
check("PVT-NAK-STOP: M8 NAK of NAV-PVT on -> no NMEA-off frame, NMEA setup "
      "within GPS_RETRY",
      want8[0] in s_on and not any(f in s_on for f in want8[1:])
      and gl._rate(200) not in s_on
      and st_on == (True, False, gl.GPS_RETRY)
      and nmea_set(again_on) == want_nmea,
      repr((st_on, [f.hex() for f in s_on])))
check("PVT-NAK-STOP: a NAK (or silence) mid-way -> the frames after it not "
      "sent",
      all(f in s_gsa for f in want8[:4])
      and not any(f in s_gsa for f in want8[4:]) and st_gsa[:2] == (True, False)
      and want8[0] in s_sil and not any(f in s_sil for f in want8[1:])
      and st_sil[:2] == (False, False),
      repr((st_gsa, st_sil, [f.hex() for f in s_sil])))

# PVT-BLIND: CFG-RATE unanswered (an overrun or booting receiver) sends the
# message frames once, blind. An M8's NAV-PVT setup is a row of CFG-MSG:
# one dropped NAV-PVT-on frame with the NMEA-off ones taken left it silent
# until a retry (checked=True skipped the "no NAV-PVT" check). Now the
# blind burst is the NMEA set, and the PVT setup follows acknowledged; an
# M10's one VALSET (all or nothing) still goes blind
saved = gl.GPS_PVT
gl.GPS_PVT = True
_uart_silent.add((0x06, 0x08))
try:
    blind8 = pvt_setup(M8_VER)
    b8_state = (gl.st["mode"], gl.st["setup_ok"])
    blind10 = pvt_setup(M10_VER)
    b10_state = (gl.st["mode"], gl.st["setup_ok"])
    blind8b = pvt_setup(M8_VER)
finally:
    _uart_silent.discard((0x06, 0x08))
try:
    _uart_tx[:] = []
    _clock[0] += gl.GPS_RETRY + 1000
    gl.gps_check(_clock[0])
    b8_next = (gl.st["mode"], gl.st["setup_ok"], gl.st["pvt_bad"],
               [f for f in _uart_tx if f[2:3] == b"\x06"])
finally:
    gl.GPS_PVT = saved
check("PVT-BLIND: M8, CFG-RATE unanswered -> the NMEA set blind, no NAV-PVT "
      "on; the acknowledged PVT setup next",
      b8_state == ("nmea", False) and want8[0] not in blind8
      and nmea_set(blind8) == want_nmea and pvt_stop in blind8
      and want8[0] not in blind8b
      and b8_next[:3] == ("pvt", True, False) and b8_next[3][2:-1] == want8
      and b10_state == ("pvt", False) and vs_frame in blind10,
      repr((b8_state, b10_state, [f.hex() for f in blind8], b8_next[:3])))

# PVT-NAK-LATE: an ACK / NAK names only class and id, and an M8's CFG-MSG
# frames are all 06 01: a late NAK of an earlier one, read as the answer
# to this one, gave PVT up for the whole boot. Now a NAK in the PVT setup
# is sent again once: refused twice is refused
nl_once = set([want8[2]])
nl_write = UART.write


def nl_nak_once(self, data):
    d = bytes(data)
    if d in nl_once:                      # this send: a NAK (a late one of
        nl_once.discard(d)                # the frame before), then ACKs
        _uart_nak_frames.add(d)
        try:
            return nl_write(self, data)
        finally:
            _uart_nak_frames.discard(d)
    return nl_write(self, data)


UART.write = nl_nak_once
try:
    nl_sent = pvt_setup(M8_VER)
    nl_state = (gl.st["mode"], gl.st["setup_ok"], gl.st["pvt_bad"])
finally:
    UART.write = nl_write
s_nak2, st_nak2, _ = m8_stop(nak=want8[2])
check("PVT-NAK-LATE: one NAK in the M8 PVT setup -> sent again, PVT kept; "
      "two -> PVT off",
      nl_state == ("pvt", True, False) and nl_sent.count(want8[2]) == 2
      and nl_sent[2:-1] == want8[:3] + want8[2:]
      and s_nak2.count(want8[2]) == 2 and st_nak2[:2] == (True, False),
      repr((nl_state, [f.hex() for f in nl_sent])))

# PVT-RESTART: a restarted receiver is back at NMEA: its banner counts, its
# NMEA positions keep the track going, and the setup turns PVT on again
# (the restarted M10 answers MON-VER: PVT-SWAP)
pvt_setup(M10_VER)
gl.st["checked"] = True
_uart_tx[:] = []
_uart_rx[:] = []
_uart_ver[0] = M10_VER
try:
    _uart_rx.extend(rv_nmea("GPTXT,01,01,02,u-blox ag - www.u-blox.com")
                    + b"\r\n")
    for k in range(3):
        _uart_rx.extend(rv_nmea("GPGGA,1200%02d.00,4730.000,N,01900.000,E,"
                                "1,08,1.0,100.0,M,40.0,M,," % k) + b"\r\n")
        _uart_rx.extend(rv_nmea("GPRMC,1200%02d.00,A,4730.000,N,01900.000,"
                                "E,0.1,,311026,,,A" % k) + b"\r\n")
    gl.read_uart()
    restart = (gl.st["gps_boots"], gl.st["fix_src"], gl.st["valid"],
               gl.st["extra"] > 0)
    for _ in range(gl.GPS_RETRY // 1000 + 1):
        _clock[0] += 1000
        gl.gps_check(_clock[0])
finally:
    _uart_ver[0] = None
check("PVT-RESTART: banner counted, NMEA fixes used, PVT set up again",
      restart == (1, "nmea", True, True) and vs_frame in _uart_tx
      and gl.st["mode"] == "pvt" and gl.st["gps_kind"] == "m10",
      repr(restart))
_uart_rx[:] = []
gl.reset()

# PVT-SWAP: the receiver's kind was kept over a restart, so a receiver
# swapped while running got the old model's setup (an M8 the M10's VALSET:
# a NAK, PVT off for the boot). Now a banner forgets the kind and asks
# MON-VER at once: the setup is the new model's, or the NMEA one (safe on
# any) until it answers
swap = []
for sw_ver in (M8_VER, None):
    pvt_setup(M10_VER)
    gl.st["checked"] = True
    _uart_tx[:] = []
    _uart_rx[:] = []
    _uart_ver[0] = sw_ver
    try:
        _uart_rx.extend(rv_nmea("GPTXT,01,01,02,u-blox ag - www.u-blox.com")
                        + b"\r\n")
        gl.read_uart()
        sw_polled = gl.MON_VER in _uart_tx
        for _ in range(gl.GPS_RETRY // 1000 + 1):
            _clock[0] += 1000
            gl.gps_check(_clock[0])
    finally:
        _uart_ver[0] = None
    sw_cfg = [f for f in _uart_tx if f[2:3] == b"\x06"]
    swap.append((sw_polled, gl.st["gps_kind"], gl.st["mode"],
                 vs_frame in sw_cfg, sw_cfg[2:-1] == want8,
                 nmea_set(sw_cfg) == want_nmea))
_uart_rx[:] = []
gl.reset()
check("PVT-SWAP: a banner asks MON-VER at once; a swapped M8 gets its own "
      "setup, an unanswered one the NMEA setup, never the M10 VALSET",
      swap == [(True, "m8", "pvt", False, True, False),
               (True, None, "nmea", False, False, True)], repr(swap))


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

    def wb_drop(w, aps_back, ok_back, after, limit=10 * 60):
        """The link drops (beacon timeout), the AP is back after `after` s:
        seconds until connected again (None within `limit`)."""
        WLAN.ok, WLAN.scan_aps = set(), []
        w._status = 5
        w._requested = True
        for i in range(limit):
            if i == after:
                WLAN.ok, WLAN.scan_aps = set(ok_back), aps_back
            _clock[0] += 1000
            gl.wifi_tick()
            gl.keepalive_tick(_clock[0])
            if i > after and gl.wifi["state"] == "connected":
                return i + 1
        return None

    # WB-1: WEB on, the home AP at -78 dBm (connects fine, under
    # ROAM_MIN_RSSI), a router restart of 60 s with nothing to send: the
    # keep-alive scans saw home only at >= ROAM_MIN_RSSI, so the radio
    # slept WIFI_SLEEP (page down 1 h); at -60 dBm it was back in 64 s
    gl.WEB, gl.WEB_AWAY, gl.PB_MODE = True, False, "scan"
    wb_mine = b"\x61" * 6
    wb_back = []
    for hid in (False, True):
        aps = [("" if hid else "home", -78, wb_mine)]
        ns = [dict(n) for n in hid_nets] if hid else rv_nets()
        w = rv_connect(["home"], aps, nets=ns)
        if hid:
            gl.wifi["home_aps"] = [wb_mine]
        for _ in range(30):
            _clock[0] += 1000
            gl.wifi_tick()
            gl.keepalive_tick(_clock[0])
        wb_back.append(wb_drop(w, aps, ["home"], 60))
    check("WB-1: a weak home AP (-78 dBm) back after a router restart wakes "
          "the radio", all(b is not None and b <= 150 for b in wb_back),
          "back after %r s" % (wb_back,))
    # under SEEN_MIN_RSSI no scan wakes it: the drop searches instead
    aps = [("home", -88, wb_mine)]
    w = rv_connect(["home"], aps)
    for _ in range(gl.WIFI_CHECK // 1000 + 2):
        _clock[0] += 1000
        gl.wifi_tick()
    rssi0 = gl.wifi["rssi"]
    back = wb_drop(w, aps, ["home"], 60)
    check("WB-1: a home link dropped under SEEN_MIN_RSSI searches on",
          rssi0 == -88 and back is not None and back <= 150,
          "%r back after %r s" % (rssi0, back))

    # WB-2: WEB_AWAY, on the phone hotspot with nothing to send: a 20 s
    # blip took the "a keep-alive scan sees home come back" shortcut,
    # but those scans only watch home networks: page down 1 h
    gl.WEB, gl.WEB_AWAY, gl.PB_MODE = True, True, "scan"
    w = rv_connect(["hot"], [("hot", -50)])
    for _ in range(60):
        _clock[0] += 1000
        gl.wifi_tick()
        gl.keepalive_tick(_clock[0])
    on_hot = gl.wifi["state"] == "connected" and gl.wifi["ssid"] == "hot"
    back = wb_drop(w, [("hot", -50)], ["hot"], 20)
    check("WB-2: WEB_AWAY: a hotspot blip searches on (not 1 h asleep)",
          on_hot and back is not None and back <= 120,
          "%r back after %r s" % (on_hot, back))
    # without WEB_AWAY there is no page on the hotspot: it sleeps as before
    gl.WEB_AWAY = False
    w = rv_connect(["hot"], [("hot", -50)])
    WLAN.ok, WLAN.scan_aps = set(), []
    w._status = 5
    w._requested = True
    for _ in range(30):
        _clock[0] += 1000
        gl.wifi_tick()
    check("WB-2: without WEB_AWAY a hotspot drop still sleeps",
          gl.wifi["state"] == "sleep", gl.wifi["state"])

    # WB-3: down_at was set by any failed round, also the hourly one after
    # a deliberate sleep (all uploaded): _web_search took that for a fresh
    # drop and searched for WEB_SEARCH (~7 extra rounds in ~35 min)
    gl.WEB, gl.WEB_AWAY, gl.PB_MODE = True, False, "gpio"
    w = rv_connect(["hot"], [("hot", -50)])
    gl.wifi["idle_at"] = _clock[0] - gl.WIFI_IDLE
    gl.wifi_tick()
    slept = gl.wifi["state"]
    WLAN.ok, WLAN.scan_aps = set(), []
    real_br = gl._wifi_begin_round
    wb_rounds = [0]

    def wb_round(w, real=real_br):
        wb_rounds[0] += 1
        return real(w)
    gl._wifi_begin_round = wb_round
    try:
        for _ in range(int(2.5 * 3600) // 5):
            _clock[0] += 5000
            gl.wifi_tick()
    finally:
        gl._wifi_begin_round = real_br
    check("WB-3: a failed round after a deliberate sleep is no drop (no "
          "WEB_SEARCH rounds)", slept == "sleep" and wb_rounds[0] <= 3
          and gl.wifi["state"] == "sleep" and gl.wifi["down_at"] is None,
          "%r %d rounds %r %r" % (slept, wb_rounds[0], gl.wifi["state"],
                                  gl.wifi["down_at"]))
    gl.WEB, gl.WEB_AWAY, gl.PB_MODE = False, False, "scan"

    # WB-4: a MemoryError after the scan (_stay_scan, _home_seen, the sort,
    # the dict copies) escaped wifi_tick and the roam: run()'s crash
    # handler reset the board. Now a failed scan (every network tried)
    w = rv_connect([], [("cafe", -60, b"\x04" * 6), ("hot", -50)])
    real_ss = gl._stay_scan

    def wb_boom(aps):
        raise MemoryError("memory allocation failed")
    gl._stay_scan = wb_boom
    wb_err = []
    try:
        gl.wifi["state"] = "not found"
        gl.wifi["last_check"] = _clock[0] - gl.wifi["backoff"] - 1
        try:
            gl.wifi_tick()
        except MemoryError as e:
            wb_err.append(("round", repr(e)))
        tried = [c["ssid"] for c in gl.wifi.get("cand") or []]
        try:
            gl._try_roam_home(w)
        except MemoryError as e:
            wb_err.append(("roam", repr(e)))
    finally:
        gl._stay_scan = real_ss
    check("WB-4: a MemoryError after a scan is a failed scan, not a reset",
          not wb_err and tried == ["home", "hot"], "%r %r" % (wb_err, tried))

    # WB-5: a good scan with no empty-SSID AP at all: a hidden network
    # cannot be in range (it beacons with an empty SSID), so it waits for
    # the UNLISTED_EVERY round like the other unlisted ones (12 s each)
    w = rv_connect([], [], nets=[dict(n) for n in hid_nets])
    w.active(True)
    order = {}
    for name, aps, rounds in (("none", [("hot", -50)], 1),
                              ("blank", [("hot", -50), ("", -80, b"\x62" * 6)],
                               1),
                              ("every", [("hot", -50)], gl.UNLISTED_EVERY),
                              ("failed", [], 1)):
        WLAN.scan_aps = aps
        gl.wifi["rounds"] = rounds
        gl.wifi["empty"] = 0
        gl.wifi["failed"] = {}
        order[name] = [c["ssid"] for c in gl._candidate_order(w)]
    check("WB-5: no blank AP in a good scan: the hidden home waits for the "
          "UNLISTED_EVERY round", order == {"none": ["hot"],
                                             "blank": ["hot", "home"],
                                             "every": ["hot", "home"],
                                             "failed": ["home", "hot"]},
          repr(order))

    # WB-6: status() raising during a roam (_wifi_poll -> _wifi_next, no
    # _mark_failed) left roaming set once the hotspot fallback failed too;
    # a later failed hidden-home try in a plain round then blamed a blank
    # AP and grew hid_wait as if it were a roam
    w = rv_connect(["hot"], [("", -50, b"\x63" * 6), ("hot", -40)],
                   nets=[dict(n) for n in hid_nets])
    _clock[0] += gl.FAIL_HOLD            # the boot round's home try
    gl._try_roam_home(w)
    roam_to = gl.wifi.get("ssid")
    WLAN.ok = set()
    wb_raise = [2]

    def wb_status(arg=None, real=w.status):
        if wb_raise[0] > 0 and arg is None:
            wb_raise[0] -= 1
            raise OSError("status")
        return real(arg)
    w.status = wb_status
    try:
        for _ in range(30):
            _clock[0] += 500
            gl.wifi_tick()
            if gl.wifi["state"] != "connecting":
                break
    finally:
        del w.status
    check("WB-6: roaming is cleared once a roam's candidates are used up",
          roam_to == "home" and gl.wifi["state"] in ("not found", "sleep")
          and gl.wifi.get("roaming") is False,
          "%r %r %r" % (roam_to, gl.wifi["state"], gl.wifi.get("roaming")))

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

    # WR3: WiFi review 3 (WR3-n: the n-th finding of the WiFi section)
    wf_saved = (gl.PB_KEEPALIVE, gl.HOME_APS_FILE, gl.DIAG, builtins.print,
                builtins.open)
    wf_said = []

    def wf_print(*a, **k):
        wf_said.append(" ".join(str(x) for x in a))

    def wf_drop(w, secs, pulse=True):
        """Beacon timeout, the AP gone for good: (rounds, last state)."""
        WLAN.ok, WLAN.scan_aps = set(), []
        w._status = 5
        w._requested = True
        real_br = gl._wifi_begin_round
        n = [0]

        def counted(w):
            n[0] += 1
            return real_br(w)
        gl._wifi_begin_round = counted
        try:
            for _ in range(secs):
                _clock[0] += 1000
                gl.wifi_tick()
                if pulse:
                    gl.keepalive_tick(_clock[0])
        finally:
            gl._wifi_begin_round = real_br
        return n[0], gl.wifi["state"]

    try:
        # WR3-1: a MemoryError in the learning scan's processing (bytes() of
        # every BSSID, the lists; a dense scan at ~15 KB free) or in the
        # HOME_APS_FILE write escaped wifi_tick: run() reset the board
        gl.WEB = False
        w = rv_connect(["home"], [("", -55, b"\x41" * 6), ("hot", -40)],
                       nets=[dict(n) for n in hid_nets])
        conn = gl.wifi["state"] == "connected" and gl.wifi["learn"]
        wf_bytes0 = builtins.bytes

        def wf_bytes(*a):
            if a and isinstance(a[0], (bytes, bytearray)) and len(a[0]) == 6:
                raise MemoryError("memory allocation failed")
            return wf_bytes0(*a)
        wf_err = []
        gl.bytes = wf_bytes
        try:
            gl.wifi_tick()
        except MemoryError as e:
            wf_err.append(("learn", repr(e)))
        finally:
            del gl.bytes
        learn_after = (gl.wifi["learn"], gl.wifi["state"])
        gl.HOME_APS_FILE = os.path.join(rv_td, "wf_home_aps")
        gl.wifi["home_aps"] = [b"\x41" * 6]

        def wf_oom_open(*a, **k):
            raise MemoryError("memory allocation failed")
        builtins.open = wf_oom_open
        try:
            gl._home_aps_save()
        except MemoryError as e:
            wf_err.append(("save", repr(e)))
        finally:
            builtins.open = wf_saved[4]
            gl.HOME_APS_FILE = None
        check("WR3-1: a MemoryError while learning the hidden home's BSSID "
              "(or saving it) is no reset", conn and not wf_err
              and learn_after == (False, "connected"),
              "%r %r %r" % (conn, wf_err, learn_after))

        # WR3-2: on the home link "home in range" still asked for
        # ROAM_MIN_RSSI (-75): at -78 dBm, associated, the veto did not
        # hold and a multipath jump ended the home stay. On the link the
        # bar is SEEN_MIN_RSSI (the one a keep-alive scan wakes it at)
        w = rv_connect(["home"], [("home", -78)])
        on78 = (gl.wifi["state"], gl.home_in_range(),
                gl._home_veto(_clock[0], {}))
        WLAN.scan_aps = [("home", -90)]
        on90 = gl.home_in_range()
        check("WR3-2: on the home link at -78 dBm home is in range (veto); "
              "at -90 it is not", on78 == ("connected", True, True)
              and on90 is False, "%r %r" % (on78, on90))

        # WR3-3: the "no page off home" exit of _web_search was in the scan
        # keep-alive branch only: in gpio / radio mode, or KEEPALIVE=0, a
        # hotspot drop (WEB_AWAY off) searched WEB_SEARCH (30 min) on
        gl.WEB, gl.WEB_AWAY = True, False
        wf3 = {}
        for mode, ka in (("gpio", 5000), ("radio", 5000), ("scan", 0)):
            gl.PB_MODE, gl.PB_KEEPALIVE = mode, ka
            w = rv_connect(["hot"], [("hot", -50)])
            on_hot = gl.wifi["ssid"] == "hot"
            wf3["%s/%d" % (mode, ka)] = (on_hot,) + wf_drop(w, 10 * 60)
        gl.PB_MODE, gl.PB_KEEPALIVE = "gpio", 5000
        w = rv_connect(["home"], [("home", -50)])
        wf3_home = wf_drop(w, 10 * 60)
        check("WR3-3: a hotspot drop without WEB_AWAY sleeps in every "
              "keep-alive mode (a home drop still searches)",
              all(v[0] and v[1] <= 1 and v[2] == "sleep"
                  for v in wf3.values())
              and wf3_home[0] >= 2 and wf3_home[1] == "not found",
              "%r %r" % (wf3, wf3_home))

        # WR3-4 / WR3-8: the same SSID twice in .env: the later entry took
        # over the scan match (its password, its home flag). The first one
        # wins, the later one is dropped with a warning; an SSIDn / WPA2_n
        # key the logger does not read and an empty WPA2 next to an SSID
        # are warned about too. Key names only, never a value
        diag_clear()
        gl.DIAG = 60000
        builtins.print = wf_print
        try:
            dup = gl._load_nets({"SSID": "home", "WPA2": "pw-new",
                                 "SSID3": "home", "WPA2_3": "pw-old"})
            dup2 = gl._load_nets({"SSID": "home", "WPA2": "pw-a",
                                  "SSID2": "hot", "WPA2_2": "pw-b",
                                  "SSID3": "hot", "WPA2_3": "pw-c",
                                  "HOME3": "1"})
            said4 = list(wf_said)
            del wf_said[:]
            unk = gl._load_nets({"SSID": "home", "WPA2": "pw-1",
                                 "SSID1": "cafe", "WPA2_1": "pw-secret",
                                 "SSID4": "bar", "WPA2_4": "pw-4",
                                 "SSID_HOME": "1", "SSID2_HIDDEN": "0"})
            said_unk = list(wf_said)
            del wf_said[:]
            opn = gl._load_nets({"SSID": "home", "WPA2": "",
                                 "SSID2": "open"})
            said_open = list(wf_said)
            del wf_said[:]
            gl._load_nets({"SSID": "home", "WPA2": "pw-1", "SSID2": "hot",
                           "WPA2_2": "pw-2", "HOME2": "0"})
            said_ok = list(wf_said)
        finally:
            builtins.print = wf_saved[3]
        wf_diag = "\n".join(diag_lines())
        WLAN.connects = []
        rv_connect([], [("home", -50)], nets=dup)
        tried = [c[1] for c in WLAN.connects]
        out4 = "\n".join(said4)
        check("WR3-4: a duplicate SSID in .env: the first entry wins, the "
              "later one is dropped with a warning (no password)",
              [(n["ssid"], n["pw"], n["home"]) for n in dup]
              == [("home", "pw-new", True)]
              and [(n["ssid"], n["pw"], n["home"]) for n in dup2]
              == [("home", "pw-a", True), ("hot", "pw-b", False)]
              and tried and set(tried) == {"pw-new"}
              and "SSID3" in out4 and "pw-" not in out4
              and "SSID3" in wf_diag and "pw-" not in wf_diag,
              "%r %r %r %r" % (dup, dup2, tried, said4))
        out_unk, out_open = "\n".join(said_unk), "\n".join(said_open)
        check("WR3-8: unknown SSIDn/WPA2_n keys and an empty WPA2 are "
              "warned about by name (no values)",
              len(unk) == 1 and len(opn) == 2
              and all(k in out_unk for k in ("SSID1", "WPA2_1", "SSID4",
                                             "WPA2_4"))
              and "SSID_HOME" not in out_unk
              and "SSID2_HIDDEN" not in out_unk
              and "pw-" not in out_unk and "cafe" not in out_unk
              and "bar" not in out_unk
              and "WPA2 " in out_open + " " and "WPA2_2" not in out_open
              and "home" not in out_open and said_ok == []
              and "SSID4" in wf_diag and "pw-" not in wf_diag,
              "%r %r %r" % (said_unk, said_open, said_ok))

        # WR3-5: _home_seeable was global: a visible home in .env made a
        # dropped hidden home whose BSSID is not learned "seeable", and
        # _web_search stopped (page down 1 h) although no keep-alive scan
        # sees that one come back. It is the dropped network's now
        gl.WEB, gl.WEB_AWAY, gl.PB_MODE, gl.PB_KEEPALIVE = (True, False,
                                                            "scan", 5000)
        mix = [{"ssid": "home", "pw": "x", "home": True, "hidden": False,
                "bssid": None, "ip": None},
               {"ssid": "hid", "pw": "z", "home": True, "hidden": True,
                "bssid": None, "ip": None}]
        w = rv_connect(["hid"], [("", -50, b"\x43" * 6)],
                       nets=[dict(n) for n in mix])
        on_hid = gl.wifi["ssid"] == "hid" and gl.wifi["state"] == "connected"
        gl.wifi["learn"] = False         # (several mesh nodes: none learned)
        gl.wifi["home_aps"] = []
        hid_drop = wf_drop(w, 120)
        w = rv_connect(["home"], [("home", -50)], nets=[dict(n) for n in mix])
        vis_drop = wf_drop(w, 120)
        check("WR3-5: a dropped hidden home (BSSID unknown) searches on "
              "though a visible home is configured",
              on_hid and hid_drop[1] == "not found"
              and gl.wifi["down_home"] is True and vis_drop[1] == "sleep",
              "%r %r %r" % (on_hid, hid_drop, vis_drop))

        # WR3-6: keep-alive scans stepped the rounds' empty-scan count: one
        # empty pulse made a round's own first empty scan (taken as failed:
        # every network tried) the second (a place without APs: nothing)
        gl.WEB, gl.PB_MODE, gl.PB_KEEPALIVE = False, "scan", 5000
        online(rv_td)
        gl.wifi["nets"] = rv_nets()
        gl.wifi["wlan"] = w = WLAN(0)
        gl.wifi["state"] = "not found"
        WLAN.scan_aps = []
        gl.wifi["pb_at"] = _clock[0] - gl.PB_KEEPALIVE
        gl.keepalive_tick(_clock[0])
        pulsed = (gl.wifi["pb_scans"], gl.wifi["empty"])
        gl.wifi["rounds"] = 1
        w.active(True)
        order = [c["ssid"] for c in gl._candidate_order(w)]
        gl.PB_CONNECTED = True
        gl.wifi.update(state="connected", empty=0,
                       pb_at=_clock[0] - gl.PB_KEEPALIVE)
        gl.keepalive_tick(_clock[0])
        conn_pulse = (gl.wifi["pb_scans"], gl.wifi["empty"])
        gl.PB_CONNECTED = False
        check("WR3-6: an empty keep-alive scan does not count as the "
              "round's empty scan", pulsed == (1, 0)
              and order == ["home", "hot"] and conn_pulse == (2, 0),
              "%r %r %r" % (pulsed, order, conn_pulse))

        # WR3-7: dead state: wifi["pw"] (a second copy of the password in
        # RAM, never read), saw_connecting, tries, pb_conn_at
        w = rv_connect(["home"], [("home", -50)])
        dead = [k for k in ("pw", "saw_connecting", "tries", "pb_conn_at")
                if k in gl.wifi]
        r2_boot("SSID=home\nWPA2=pw-x\n", [("home", -50)], ["home"])
        r2_until(lambda: gl.wifi["state"] == "connected", 30)
        dead += [k for k in ("pw", "saw_connecting", "tries", "pb_conn_at")
                 if k in gl.wifi]
        check("WR3-7: no dead WiFi state (no password copy outside nets)",
              gl.wifi["state"] == "connected" and dead == [], repr(dead))
    finally:
        (gl.PB_KEEPALIVE, gl.HOME_APS_FILE, gl.DIAG, builtins.print,
         builtins.open) = wf_saved
        if hasattr(gl, "bytes"):
            del gl.bytes
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


def rb_fail(path, off=0, key=None):
    if path == bad_p:
        raise OSError(5)
    return real_rb(path, off, key)
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

# UPL-7: the points of a track parked as .err leave up["left"] (counted at
# boot, they were never sent: 'left' stayed above 0 and held the OTA)
online(up_td)
gl.up["url"], gl.up["key"] = "https://d.example", "k"
make_trk(bad_p, 5)
make_trk(ok_p, 3)
gl.up["left"] = gl.count_points()
u7_left0 = gl.up["left"]
gl.read_batch = rb_fail
try:
    for _ in range(gl.READ_ERR_MAX + 4):
        gl.up["next_at"] = _clock[0]
        for p_ in list(gl.up["hold"]):
            gl.up["hold"][p_] = _clock[0] - 1
        gl.upload_tick()
        _clock[0] += 1000
finally:
    gl.read_batch = real_rb
check("UPL-7: a track parked as .err after read errors leaves 'left'",
      u7_left0 == 8 and os.path.exists(bad_p + ".err")
      and not os.path.exists(ok_p) and gl.up["left"] == 0,
      repr((u7_left0, gl.up["left"], os.listdir(up_td))))
os.remove(bad_p + ".err")
# ... and one parked because it cannot be deleted after its last batch
make_trk(ok_p, 3)
gl.up["left"] = gl.count_points()
_rm7 = os.remove
os.remove = lambda path, *a: (_ for _ in ()).throw(OSError(5)) \
    if path == ok_p else _rm7(path, *a)
try:
    for _ in range(gl.READ_ERR_MAX + 1):
        gl.up["hold"] = {}
        pump()
        _clock[0] += 1000
finally:
    os.remove = _rm7
check("UPL-7: a track parked as .err after failed deletes leaves 'left'",
      os.path.exists(ok_p + ".err") and gl.up["left"] == gl.count_points()
      == 0, repr((gl.up["left"], os.listdir(up_td))))
os.remove(ok_p + ".err")

# UPL-4: a 5xx or another answer that is no payload error (404, 403, 3xx)
# for the same batch SRV_ERR_MAX times in a row holds that track, so the
# others go (only a global backoff: files[0] retried for ever, the queue
# blocked)
u4 = []
for code_ in (500, 404):
    online(up_td)
    gl.up["url"], gl.up["key"] = "https://d.example", "k"
    make_trk(bad_p, 3)
    make_trk(ok_p, 4)
    _post_code[0] = code_
    for _ in range(getattr(gl, "SRV_ERR_MAX", 3)):
        pump()
        _clock[0] += 1000
    held4 = bad_p in (gl.up.get("hold") or {})
    _post_code[0] = 201
    _posts[:] = []
    pump()
    u4.append((code_, held4, len(locations()), os.path.exists(bad_p),
               os.path.exists(ok_p)))
    gl.up["hold"] = {}
    pump()
    u4[-1] += (os.path.exists(bad_p),)
check("UPL-4: SRV_ERR_MAX server errors on one batch hold that track, the "
      "next one goes, then the held one",
      u4 == [(500, True, 4, True, False, False),
             (404, True, 4, True, False, False)], repr(u4))

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

# POS-BIG: a corrupt, oversized .pos is read bounded (resent from 0), and a
# MemoryError reading it is "low mem" like the other read errors, not an
# exception out of upload_tick (a reset after every WiFi connect)
pb_p = os.path.join(up_td, "20261004_130000.trk")


class _PosSized(object):
    """A .pos whose whole read() does not fit in the heap."""
    def __init__(self, data):
        self.data = data

    def read(self, n=-1):
        if n is None or n < 0 or n > 256:
            raise MemoryError()
        return self.data[:n]

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False


pb_mode = ["big"]


def _pb_open(name, mode="r", *a, **k):
    if name == pb_p + ".pos" and mode in ("r", "rb"):
        if pb_mode[0] == "oom":
            raise MemoryError()
        with builtins.open(name, mode) as f_:
            return _PosSized(f_.read())
    return builtins.open(name, mode, *a, **k)


pb_res = []
gl.open = _pb_open
try:
    for mode_ in ("big", "oom"):
        pb_mode[0] = mode_
        make_trk(pb_p, 3)
        with builtins.open(pb_p + ".pos", "w") as f:
            f.write("1" * 100000)
        _posts[:] = []
        gl.up["hold"] = {}
        try:
            pump()
            crash = None
        except MemoryError as e:
            crash = e
        pb_res.append((crash is None, len(locations()), gl.up["state"],
                       os.path.exists(pb_p)))
        _clock[0] += 1000
finally:
    del gl.open
check("POS-BIG: an oversized .pos is read bounded: resent from 0, no crash",
      pb_res[0] == (True, 3, "ok", False), repr(pb_res))
check("POS-BIG: a MemoryError reading the .pos is 'low mem', no crash",
      pb_res[1] == (True, 0, "low mem", True), repr(pb_res))
for n_ in os.listdir(up_td):
    os.remove(os.path.join(up_td, n_))

# POS-EIO: a .pos that is there but unreadable (EIO) is no offset 0: the
# track waits (the newer ones go) instead of resending what is up (and a
# .trb's history twice); after READ_ERR_MAX tries it resends from 0 after all
online(up_td)
gl.up["url"], gl.up["key"] = "https://d.example", "k"
pe1 = os.path.join(up_td, "20261004_180000.trk")
pe2 = os.path.join(up_td, "20261004_190000.trk")
make_trk(pe1, gl.UPLOAD_BATCH * 2)
make_trk(pe2, 3)
pump()                                   # pe1's first batch up, .pos written


def _pe_open(name, mode="r", *a, **k):
    if name == pe1 + ".pos" and mode in ("r", "rb"):
        raise OSError(5)
    return builtins.open(name, mode, *a, **k)


def _pe_sizes():
    return [len(json.loads(p_["data"].decode())["locations"])
            for p_ in _posts]


_posts[:] = []
gl.open = _pe_open
try:
    _clock[0] += 1000
    pump()
    pe_eio = (gl.up["state"], _pe_sizes())
    _clock[0] += 1000
    pump()                               # pe1 held: pe2 goes
    pe_next = (_pe_sizes(), os.path.exists(pe2))
finally:
    del gl.open
_posts[:] = []
gl.up["hold"] = {}
_clock[0] += 1000
pump()                                   # readable again: resumes
pe_back = (_pe_sizes(), locations()[0]["geometry"]["coordinates"][1])
make_trk(pe1, gl.UPLOAD_BATCH * 2)
gl.write_offset(pe1, os.path.getsize(pe1) // 2)
_posts[:] = []
gl.open = _pe_open
try:
    for _ in range(gl.READ_ERR_MAX):
        gl.up["hold"] = {}
        _clock[0] += 1000
        pump()
finally:
    del gl.open
pe_max = _pe_sizes()
check("POS-EIO: an unreadable .pos holds the track, no resend from 0",
      pe_eio == ("fs err", []) and pe_next == ([3], False)
      and pe_back == ([gl.UPLOAD_BATCH],
                      round(52.5 + gl.UPLOAD_BATCH * 0.0001, 6)),
      "%r %r %r" % (pe_eio, pe_next, pe_back))
check("POS-EIO: after READ_ERR_MAX unreadable .pos tries it resends from 0",
      pe_max == [gl.UPLOAD_BATCH], repr(pe_max))
# POS-EIO-COUNT: count_points with that .pos unreadable counted the track
# as 0 waiting points (the OSError caught per file), while it still waits
# and may go up from 0: count it whole
make_trk(pe1, 30)
gl.write_offset(pe1, os.path.getsize(pe1) // 2)
gl.open = _pe_open
try:
    pc_n = gl.count_points()
finally:
    del gl.open
check("POS-EIO-COUNT: an unreadable .pos counts the whole track, not 0",
      pc_n == 30, repr(pc_n))
for n_ in os.listdir(up_td):
    os.remove(os.path.join(up_td, n_))

# UPL-1: a .pos that is not UTF-8 (a damaged flash block) is a corrupt one,
# resent from 0. Read as text it raised UnicodeError (a ValueError) out of
# read_pos, count_points, up_reset and upload_tick: a reset at every boot
online(up_td)
u1_p = os.path.join(up_td, "20261004_200000.trk")
make_trk(u1_p, 5)
with builtins.open(u1_p + ".pos", "wb") as f:
    f.write(b"\xff\xfe 12\n")
u1_res = []
for fn_ in (lambda: gl.read_pos(u1_p), gl.count_points, gl.up_reset):
    try:
        u1_res.append(fn_())
    except Exception as e:
        u1_res.append(type(e).__name__)
u1_left = gl.up.get("left")
gl.up["url"], gl.up["key"] = "https://d.example", "k"
gl.wifi["state"] = "connected"
_posts[:] = []
try:
    pump()
    u1_up = (len(locations()), os.path.exists(u1_p))
except Exception as e:
    u1_up = type(e).__name__
check("UPL-1: a .pos that is not UTF-8 is corrupt: offset 0, counted, "
      "resent, no exception",
      u1_res == [(0, None), 5, None] and u1_left == 5 and u1_up == (5, False),
      repr((u1_res, u1_left, u1_up)))
# any other error counting one track (not only OSError / MemoryError)
# leaves the others counted: it ran in up_reset at boot
make_trk(u1_p, 5)
make_trk(os.path.join(up_td, "20261004_210000.trk"), 7)
real_rp = gl.read_pos
gl.read_pos = lambda p: ((_ for _ in ()).throw(ValueError("x"))
                         if p == u1_p else real_rp(p))
try:
    u1_cnt = gl.count_points()
except Exception as e:
    u1_cnt = type(e).__name__
finally:
    gl.read_pos = real_rp
check("UPL-1: count_points survives any error of one track",
      u1_cnt == 7, repr(u1_cnt))
for n_ in os.listdir(up_td):
    os.remove(os.path.join(up_td, n_))

# HTTP-NOGATE: the IDF block and Python heap margins guard TLS: a plain
# http:// GEO_URL posts with both low, and never reboots for them
online(up_td)
gl.up["url"], gl.up["key"] = "http://192.168.1.139:3000", "k"
hn_p = os.path.join(up_td, "20261004_140000.trk")
make_trk(hn_p, gl.UPLOAD_BATCH * 2)
hn_saved = (gl.idf_heap, gl._idf_info, gl.gc.mem_free)
gl.idf_heap = lambda: (30, 20)
gl._idf_info = lambda: (30 * 1024, 20 * 1024)
gl.gc.mem_free = lambda: 20 * 1024 + 2000
gl.wifi["home"] = True
_resets[:] = []
try:
    for _ in range(3):
        try:
            pump()
        except SystemExit:
            break
        _clock[0] += gl.LOWIDF_WAIT
finally:
    gl.idf_heap, gl._idf_info, gl.gc.mem_free = hn_saved
    gl.wifi["home"] = False
check("HTTP-NOGATE: plain http posts with low IDF and Python heaps, no reboot",
      len(_posts) == 2 and not _resets and not os.path.exists(hn_p),
      "posts %d resets %d state %s" % (len(_posts), len(_resets),
                                       gl.up["state"]))
for n_ in os.listdir(up_td):
    os.remove(os.path.join(up_td, n_))

# LEFT-BAD: points moved into a .bad leave "left" once; uploading the .bad
# later does not take them off again (count_points never counts a .bad)
online(up_td)
gl.up["url"], gl.up["key"] = "https://d.example", "k"
lb1 = os.path.join(up_td, "20261004_150000.trk")
lb2 = os.path.join(up_td, "20261004_160000.trk")
make_trk(lb1, gl.UPLOAD_BATCH * 2)
make_trk(lb2, gl.UPLOAD_BATCH * 2)
gl.up["left"] = gl.count_points()
pump()                                   # lb1's first batch up
_post_code[0] = 422
_clock[0] += 1000
pump()                                   # its second to lb1.bad
_post_code[0] = 201
lb_q = (gl.up["left"], gl.count_points(), os.path.exists(lb1 + ".bad"))
gl.up["hold"] = {lb2: _clock[0] + gl.BACKOFF_MAX // 2}   # the .bad first
gl.up["allow_bad"] = True
for _ in range(3):
    _clock[0] += 1000
    pump()
lb_b = (os.path.exists(lb1 + ".bad"), dict(gl._web_rows())["left"],
        gl.count_points())
check("LEFT-BAD: a quarantined batch leaves 'left' once, not again when "
      "its .bad goes up",
      lb_q == (gl.UPLOAD_BATCH * 2, gl.UPLOAD_BATCH * 2, True)
      and lb_b == (False, gl.UPLOAD_BATCH * 2, gl.UPLOAD_BATCH * 2),
      "%r %r" % (lb_q, lb_b))
for n_ in os.listdir(up_td):
    os.remove(os.path.join(up_td, n_))

# BAD-IO: a .bad copy that fails for another reason than a full .bad (an
# I/O error, the flash reserve) is "fs err", not BAD FULL. BAD-FULL: a full
# .bad is BAD FULL only while that .bad is on flash
online(up_td)
gl.up["url"], gl.up["key"] = "https://d.example", "k"
bi_p = os.path.join(up_td, "20261004_170000.trk")
make_trk(bi_p, 3)


def _bi_open(name, mode="r", *a, **k):
    if name == bi_p + ".bad" and "a" in mode:
        raise OSError(5)
    return builtins.open(name, mode, *a, **k)


def _bi_state():
    return (gl.up["state"], gl._bad_full_left(), gl._mode_line())


_post_code[0] = 422
bi_res = []
gl.open = _bi_open
try:
    pump()
    bi_res.append(_bi_state())
finally:
    del gl.open
real_free_ = gl.free
gl.free = lambda: gl.MIN_FREE            # the flash reserve, not the .bad
try:
    gl.up.update(hold={}, fail4=0)
    _clock[0] += 1000
    pump()
    bi_res.append(_bi_state())
finally:
    gl.free = real_free_
with builtins.open(bi_p + ".bad", "wb") as f:
    f.write(b"x" * gl.MAX_BAD_BYTES)
gl.up.update(hold={}, fail4=0)
_clock[0] += 1000
pump()
bf_full = _bi_state()
os.remove(bi_p + ".bad")                 # pulled: the track waits no more
bf_gone = (gl._bad_full_left(), os.path.exists(bi_p))
_post_code[0] = 201
check("BAD-IO: an I/O error or the flash reserve copying to .bad is "
      "'fs err', not BAD FULL",
      bi_res == [("fs err", False, "Mode LOG+UP")] * 2, repr(bi_res))
check("BAD-FULL: a full .bad is BAD FULL only while that .bad is on flash",
      bf_full == ("bad full", True, "Mode BAD FULL")
      and bf_gone == (False, True), "%r %r" % (bf_full, bf_gone))
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
               and sent.startswith(b"POST /sub HTTP/1.0\r\n"
                                   b"Host: d.example:8443\r\n")
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

# HOST-PORT: the Host header names a port that is not the scheme's default
# (RFC 7230 5.4: a proxy or virtual host on :8443 routes by it)
gl.socket, gl.select = fake_sockmod, fake_sel
sys.modules["tls"] = RvTls
hp_hosts = []
try:
    for u_ in ("https://d.example:8443/sub", "https://d.example/sub",
               "https://d.example:443/sub", "http://h.lan:3000/x",
               "http://h.lan:80/x"):
        rv_sock[0] = RvSock(b"HTTP/1.1 201 Created\r\n\r\n", polls=0)
        gl._raw_post(u_, b"{}", {}, 15)
        hp_hosts.append(rv_sock[0].out.split(b"\r\n")[1])
finally:
    gl.socket, gl.select = real_mods[0], real_mods[1]
    if real_mods[2] is None:
        sys.modules.pop("tls", None)
    else:
        sys.modules["tls"] = real_mods[2]
    tls_ctx_reset()
check("HOST-PORT: POST Host header has a non-default port, not a default one",
      hp_hosts == [b"Host: d.example:8443", b"Host: d.example",
                   b"Host: d.example", b"Host: h.lan:3000", b"Host: h.lan"],
      repr(hp_hosts))

# UPL-9: an API_KEY or GEO_URL that cannot go into a request head (a space,
# CR or another control byte, non-ASCII) or whose host:port does not parse
# (userinfo, IPv6, a port that is no number: int() raised, "err net" for
# ever) sends nothing: "no .env" and a diag line that names the key, never
# its value
u9_td = tempfile.mkdtemp(prefix="env9-")
u9 = []
diag_clear()
gl.DIAG = 60000
try:
    for url_, key_ in (("https://d.example", "ab cd"),
                       ("https://d.example", "abc\r\nX-Evil: 1"),
                       ("https://d.example", "ab\x01cd"),
                       ("https://d.example", u"kulcs\xe9"),
                       ("https://user:pw@d.example", "k"),
                       ("https://[::1]:8443", "k"),
                       ("https://d.example:80a", "k"),
                       ("https://d.example:99999", "k"),
                       ("https://d.ex ample/x", "k"),
                       ("ftp://d.example", "k"),
                       ("https://d.example:8443/sub", "good-Key_1.~")):
        online(u9_td)
        gl.up["url"], gl.up["key"] = url_, key_
        make_trk(os.path.join(u9_td, "20261009_100000.trk"), 3)
        try:
            pump()
            u9.append((len(_posts), gl.up["state"]))
        except Exception as e:
            u9.append(type(e).__name__)
finally:
    gl.DIAG = 0
    shutil.rmtree(u9_td, ignore_errors=True)
u9_diag = [l for l in diag_lines() if "env" in l]
diag_clear()
check("UPL-9: a bad API_KEY / GEO_URL sends nothing ('no .env'), a good "
      "one goes",
      u9 == [(0, "no .env")] * 10 + [(1, "ok")], repr(u9))
check("UPL-9: one diag line per bad value, naming the key, not the value",
      len(u9_diag) == 10 and all(("API_KEY" in l) != ("GEO_URL" in l)
                                 for l in u9_diag)
      and not any(s_ in l for l in u9_diag
                  for s_ in ("ab cd", "abc", "ab\x01", "kulcs", "user:",
                             "pw@", "::1", "80a", "99999", "ex ample",
                             "ftp")),
      repr(u9_diag))


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

# UPL-3: a CA_FILE over CA_MAX (a PEM bundle saved as ca.der) is not read:
# "ca err", not "low mem" (its f.read() hit MemoryError at every POST, a
# TLS reboot every few minutes for ever)
ca_td = tempfile.mkdtemp(prefix="ca-")
ca_p = os.path.join(ca_td, "ca.der")
with open(ca_p, "wb") as f:
    f.write(b"\x30\x82" + b"x" * getattr(gl, "CA_MAX", 200000))
ca_reads = []


def _ca_open(name, mode="r", *a, **k):
    if name == ca_p:
        ca_reads.append(mode)
        raise MemoryError()
    return builtins.open(name, mode, *a, **k)


online(up_td)
gl.CA_FILE = ca_p
gl.open = _ca_open
tls_ctx_reset()
try:
    gl.up["enomem"] = 0
    ca_big = (gl._tls_ready(_clock[0]), gl.up["state"], gl.up["enomem"],
              ctx_now(), ca_reads)
finally:
    del gl.open
    gl.CA_FILE = "/ca.der"
    tls_ctx_reset()
    shutil.rmtree(ca_td, ignore_errors=True)
check("UPL-3: a CA_FILE over CA_MAX is not read: 'ca err', not the ENOMEM "
      "run", ca_big == (False, "ca err", 0, None, []), repr(ca_big))

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

# UPL-3: the TLS reboot is rate limited as the low-memory one: it writes
# LOWIDF_MARK, and after a boot it took, none before LOWIDF_REBOOT_GAP of
# uptime (a lasting cause, a CA_FILE too big for the heap or a chain over
# TLS_MIN_KB, rebooted every ~2.5 min for ever)
u3_mark = os.path.exists(gl.LOWIDF_MARK)
online(up_td)                            # the next boot takes the mark
gl.up["url"], gl.up["key"] = "https://d.example", "k"
gl.wifi["home"] = True
make_trk(os.path.join(up_td, "20261006_100000.trk"), 3)
u3_saved = list(gl._up)
gl._up[:] = [0, _clock[0], 0]           # just booted
u3_boot = gl.up.get("lowidf_boot")
_resets[:] = []
u3_t = None
t0_ = _clock[0]
try:
    while _clock[0] - t0_ < gl.LOWIDF_REBOOT_GAP + 4 * gl.LOWIDF_RETRY:
        _post_raise[0] = OSError(-0x7F00)
        try:
            pump()
        except SystemExit:
            u3_t = _clock[0] - t0_
            break
        _clock[0] += gl.LOWIDF_RETRY
finally:
    _post_raise[0] = None
    gl.wifi["home"] = False
    gl._up[:] = u3_saved
check("UPL-3: a TLS reboot writes LOWIDF_MARK, the next waits "
      "LOWIDF_REBOOT_GAP of uptime",
      u3_mark and u3_boot is True and u3_t is not None
      and gl.LOWIDF_REBOOT_GAP <= u3_t <= gl.LOWIDF_REBOOT_GAP
      + gl.LOWIDF_RETRY and len(_resets) == 1,
      repr((u3_mark, u3_boot, u3_t, len(_resets))))
if os.path.exists(gl.LOWIDF_MARK):
    os.remove(gl.LOWIDF_MARK)

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
gl._up[:] = [0, _clock[0], 0]           # just booted
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

# LOWPY-REBOOT: the Python heap low at home for LOWIDF_WAIT with tracks
# waiting reboots cleanly as a low IDF heap does (LOWIDF_MARK, at most one
# per LOWIDF_REBOOT_GAP); away it only waits


def _lp_run(home_, limit):
    """ms of low Python heap until the reboot, None if none in limit."""
    gl.wifi["home"] = home_
    t0 = _clock[0]
    while _clock[0] - t0 <= limit:
        try:
            pump()
        except SystemExit:
            return _clock[0] - t0
        _clock[0] += retry_ms
    return None


if os.path.exists(mark):
    os.remove(mark)
gl._idf_info = lambda: (157 * 1024, 126 * 1024)
gl.idf_heap = lambda: (157, 126)
gl.gc.mem_free = lambda: 126 * 1024 + 4000
saved_up = list(gl._up)
diag_clear()
gl.DIAG = 60000
_resets[:] = []
try:
    online(up_td)
    gl.up["url"], gl.up["key"] = "https://d.example", "k"
    make_trk(lp_p, 3)
    lpr_away = _lp_run(False, wait_ms + 4 * retry_ms)
    online(up_td)
    gl.up["url"], gl.up["key"] = "https://d.example", "k"
    make_trk(lp_p, 3)
    lpr_home = _lp_run(True, wait_ms + 4 * retry_ms)
    lpr_mark = os.path.exists(mark)
    lpr_diag = [l for l in diag_lines() if l.startswith("reboot low")]
    online(up_td)                        # the next boot takes the mark
    gl.up["url"], gl.up["key"] = "https://d.example", "k"
    make_trk(lp_p, 3)
    gl._up[:] = [0, _clock[0], 0]       # just booted
    lpr_gap = _lp_run(True, gap_ms + 4 * retry_ms)
finally:
    gl.gc.mem_free = real_mem_free
    gl.idf_heap = real_idf
    if real_idf_info is None:
        del gl._idf_info
    else:
        gl._idf_info = real_idf_info
    gl.DIAG = 0
    gl.wifi["home"] = False
    gl._up[:] = saved_up
diag_clear()
if os.path.exists(mark):
    os.remove(mark)
check("LOWPY-REBOOT: low Python heap at home reboots after LOWIDF_WAIT "
      "(mark, diag line), away never",
      lpr_away is None and lpr_home is not None
      and wait_ms <= lpr_home <= wait_ms + retry_ms and lpr_mark
      and lpr_diag == ["reboot low mem 4000 B %d s" % (lpr_home // 1000)],
      "%r %r %r %r" % (lpr_away, lpr_home, lpr_mark, lpr_diag))
check("LOWPY-REBOOT: at most one low-heap reboot per LOWIDF_REBOOT_GAP",
      lpr_gap is not None and gap_ms <= lpr_gap <= gap_ms + retry_ms,
      repr(lpr_gap))
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


def _rb_grows(path, off=0, key=None):
    heap_kb[0] = 248                     # the read grew the heap
    return real_rb_(path, off, key)


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
gl.st.update(rx=_clock[0], pos_at=_clock[0], valid=True, fixtype=3, quality=1,
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
gl.st.update(pos_at=_clock[0], valid=True, fixtype=3, quality=1, hdop=None, lat=47.1,
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
import re
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
          "NMEA count",
          "<small>UBX BAD</small><b>0.40<em>%</em>" in tm3
          and "NMEA BAD" not in tm3 and "<small>NMEA BAD</small>" in tm,
          tm3[-300:])
    # BOOT: when this run started (local time) and how long the one before
    # it lasted, until when; FIRMWARE: the build date, the commit after it
    jb = json.loads(page_json)
    now_ = time.time()
    jb.update(uptime=10492, boot_cause=2, prev_uptime=150,
              prev_run="2 min 30 s (last line x)",
              prev_end=time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                     time.gmtime(now_ - 10492 - 30)),
              fw="86fb9c8", fw_build=1793548800, ota="current")
    rb_ = subprocess.run([node, os.path.join(here, "web_page_test.js"),
                          os.path.join(here, "..", "web.html")],
                         input=json.dumps(jb), capture_output=True, text=True)
    syb = json.loads(rb_.stdout).get("sys", "")

    def _hm(t):
        return time.strftime("%H:%M", time.localtime(t))

    boots = ['<tr><td>Running</td><td>2 h 54 min<span class="sx-chip">since '
             '%s</span></td></tr><tr><td>Last boot</td><td>hard reset</td>'
             '</tr><tr><td>Previous run</td><td>2 min 30 s, until %s</td>'
             '</tr>' % (_hm(x), _hm(x - 30))
             for x in (now_ - 10492, now_ - 10492 - 60)]
    fwl = ('<i></i>Firmware <em>%s</em></span><b>up to date</b>'
           % time.strftime("%Y-%m-%d %H:%M", time.localtime(1793548800)))
    check("PAGE-BOOT: Device shows the uptime with the boot's local time, "
          "the cause and the previous run up to when; Firmware the build "
          "date and 'up to date', the commit in the subtitle",
          any(b_ in syb for b_ in boots) and fwl in syb
          and "health &middot; 86fb9c8</p>" in syb
          and json.loads(rb_.stdout).get("unlabeled") == [],
          syb[syb.find("Firmware"):syb.find("Chip")])
    check("PAGE-PVT: Systems shows the fix source and the UBX frame counts",
          "&middot; NAV-PVT 500/2 ubx" in sy3 and v3.get("unlabeled") == []
          and " ubx" not in v.get("sys", ""), sy3[:1500])
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
          and "NO SATELLITE LIST" in v2.get("sig", ""), tm2[:400])
    check("PAGE: sky radar - horizon east at x 90, zenith centre, hollow unused",
          '<circle class="g0" cx="90" cy="0" r="3.4"><title>GPS 16 26 dBHz'
          in v.get("dots", "")
          and '<circle class="g3 u" cx="0" cy="0" r="2.4">' in v.get("dots", "")
          and "SBAS 5" not in v.get("dots", "-")
          and v.get("skyh") == "Sky &middot; 1 locked / 3 tracked",
          v.get("dots", "")[:300])
    # PAGE-SIGNAL: in words, not dB-Hz (Bator: "what is a dB-Hz? in human
    # language"): an overall grade, satellites in use of those in view,
    # how many are strong / fair / weak; a bar a satellite, strongest
    # first, its number under it, a hollow one not in use; zones named
    # strong (35+), fair (25-34), weak; it replaced the Static test
    sg = v.get("sig", "")
    check("PAGE-SIGNAL: overall grade and counts in words, a bar a "
          "satellite (strongest first, hollow when not in use), zones named",
          '<b class="q-fair">Fair<em>overall</em></b>' in sg
          and "1<em>satellite in use, of 3 in view</em>" in sg
          and "0 strong &middot; 1 fair &middot; 0 weak" in sg
          and sg.index('class="sg-b q-fair"') < sg.index('class="sg-b sg-u')
          and "<span>16</span>" in sg and "dB" not in sg
          and all("<em>%s</em>" % z in sg for z in ("strong", "fair", "weak")),
          sg[:400])
    sy = v.get("sys", "")
    check("PAGE: Systems - every JSON key shown, escaped; readiness, "
          "storage, uplink and device sections; a silent GPS link an error",
          v.get("unlabeled") == [] and "System readiness</span><b>60%</b>" in sy
          and '<div class="sx-seg bad">' + "<i></i>" * 6 in sy
          and "Storage module" in sy and "Uplink" in sy and "Device" in sy
          and "&lt;x&gt;" in sy and "<x>" not in sy
          and "Upload <em>idle" in sy and "GPS receiver <em>" in sy
          and '<div class="sx-r bad"><span><i></i>GPS link <em>no frame yet'
              in sy and "<b>Error</b>" in sy
          and '<div class="sx-r "><span><i></i>Memory <em>IDF 144 KB of 157'
              in sy,
          repr(v.get("unlabeled")) + sy[sy.find("Upload <em>") - 300:
                                         sy.find("Storage module")])
    # PAGE-EV: the Event log aggregates the whole diag log on the board
    # (tests/web_page_test.js feeds a canned 3 h log, Budapest time)
    ev = v.get("ev", {})
    evw = ev.get("view", {})
    check("PAGE-EV: the harness ran the Event log (no error)",
          "error" not in ev and evw, repr(ev.get("error", ""))[:600])
    check("PAGE-EV: header - the window in local time and its length",
          evw.get("evs") == "10:00&ndash;12:59 &middot; 2 h 59 min of the "
                            "diag log", repr(evw.get("evs")))
    evk = evw.get("evk", "")
    check("PAGE-EV: KPI tiles - 3D fix % of the status lines (2 after a "
          "boot without utc counted), no fix as a duration, boots with "
          "power-on, updates once each with the failures",
          ev.get("lines") == 166
          and '<div class="t-ok"><small>3D fix</small><b>81<em>% of minutes'
              '</em></b></div>' in evk
          and '<div class="t-warn"><small>No fix / no data</small><b>32<em>'
              'min</em></b></div>' in evk
          and "<small>Boots</small><b>3<em>1 power-on</em></b>" in evk
          and '<div class="t-warn"><small>Updates</small><b>2<em>1 failed'
              '</em></b>' in evk, evk)
    ev_n = {"crash": 1, "bug": 2, "gloop": 4, "gsil": 2, "track": 1,
            "full": 1, "upl": 3, "ota": 2, "otax": 1, "reboot": 1,
            "tback": 1, "wfail": 4, "wlost": 1, "low": 1, "setup": 4,
            "pvt": 1, "env": 2, "boot": 2, "wback": 1, "stay": 1, "grx": 2,
            "other": 2}
    # the latest first; at the same last time by severity, then count
    ev_order = ["boot", "grx", "ota", "upl", "bug", "crash", "track", "full",
                "setup", "env", "otax", "reboot", "tback", "wlost", "low",
                "pvt", "wback", "stay", "other", "gsil", "gloop", "wfail"]
    lms = ev.get("lastMs") or []
    check("PAGE-EV: groups - counts by kind ('(+N more)', '(N more)' and "
          "'N more windows' counted, an update once however many lines), "
          "the 2 lines after the last status line pending and shown, rows "
          "by the last time seen (latest first), then severity and count",
          ev.get("n") == ev_n and ev.get("pend") == 2
          and ev.get("order") == ev_order
          and len(lms) == len(ev_order) and None not in lms
          and all(lms[i] >= lms[i + 1] for i in range(len(lms) - 1))
          and lms[0] > lms[-1]
          and ev.get("viewN") == [ev_n[k] + (k in ("boot", "grx"))
                                  for k in ev_order],
          repr((ev.get("n"), ev.get("pend"), ev.get("order"),
                ev.get("viewN"))))
    det = ev.get("det", {})
    check("PAGE-EV: details - silent time summed from 'gps back after N s', "
          "the updates in order with the rolled back one, WiFi SSID, status "
          "and mean time, boots, reset loop, Other",
          "3 min 20 s in total" in det.get("gsil", "")
          and "a3f2271 &rarr; 69f8c15" in det.get("ota", "")
          and "1 rolled back" in det.get("ota", "")
          and det.get("wfail", "").startswith(
              "krumpli &middot; status 201<em> &middot; 12 s each")
          and det.get("boot", "").startswith(
              "1 power-on &middot; 2 reset<em> &middot; longest run 7 h 26 min")
          and det.get("gloop", "").startswith("5 restarts in 6 s")
          and det.get("other", "") == "clock set, heap grew"
          and "3 points dropped" in det.get("full", ""), repr(det))
    sp = ev.get("spark", {})
    check("PAGE-EV: per-hour sparkline buckets over the window (local "
          "10, 11, 12 h), each event at the next status line's time, the "
          "pending ones at the last; last seen",
          sp.get("wfail") == [4, 0, 0] and sp.get("gsil") == [0, 2, 0]
          and sp.get("boot") == [1, 1, 1] and sp.get("ota") == [0, 1, 1]
          and sp.get("gloop") == [4, 0, 0]
          and ev.get("last", {}).get("wfail") == "10:31"
          and ev.get("last", {}).get("boot") == "12:59"
          and ev.get("last", {}).get("ota") == "12:11",
          repr((sp, ev.get("last"))))
    # PAGE-TELE: the satellites curve was there twice (the Signal panel);
    # in its place GPS telemetry, after the "Orbital Systems" shot Bator
    # picked: position accuracy as a dot matrix (a column a time slice,
    # taller = smaller HDOP, red = no fix), the receiver link (sentences a
    # second from each status line's NMEA count: a minute where it stood
    # still is a drop-out) and an anomaly card while there was one
    evt = evw.get("evt", "")
    check("PAGE-TELE: accuracy share and matrix, link up time, broken "
          "sentences, one 9 min drop-out in an anomaly card",
          "GPS telemetry" in evt and evt.count('<i class="gt-c') == 36
          and "<b>56%</b>" in evt and "typical 1.2" in evt
          and '<b class="gt-n">2<em>h</em> 33<em>min</em></b><small>the receiver sent data</small>' in evt
          and '<b class="gt-n a">9<em>min</em></b><small>it sent nothing (1 drop-out)</small>' in evt
          and "<b>5</b> broken messages &middot; 1 in 18\u202f361" in evt
          and "of minutes" not in evt
          and 'class="gt-chip a">Anomaly' in evt and "evc" not in evw,
          evt[:900])
    # (Bator: "with no scale, how do I know when it happened?") the link
    # is a strip under the accuracy matrix on its time axis, a cell a
    # column (red where no data came), and the anomaly card lists each
    # drop-out with its local time and length, newest first
    check("PAGE-TELE: the link strip shares the matrix's columns and time "
          "axis, a marker with the start time under each drop-out; the card "
          "heads its list with their total",
          evt.count('<i class="gt-k') == 36
          and evt.count('<i class="gt-k rd"') >= 1
          and "<span>11:00&ndash;11:09</span><b>9 min</b>" in evt
          and "<h4><i>!</i>9 min without data, in 1 drop-out:</h4>" in evt
          and '<span class="gt-mk" style="left:' in evt
          and '"><i></i>11:00</span>' in evt
          and "<svg" not in evt, evt[-1500:])
    evg = evw.get("evg", "")
    check("PAGE-EV: the grouped table - a header row, a row a kind with its "
          "icon chip, name, count, detail, sparkline and last seen",
          evg.startswith('<div class="ev-grp ev-hd">')
          and evg.count('<div class="ev-grp">') == 22
          and '<div class="ev-grp"><span class="ev-ic t-bad"><svg' in evg
          and '<span class="ev-nm t-bad">GPS reset loop</span><span '
              'class="ev-n">4</span>' in evg
          and '<span class="ev-lt">10:46</span>' in evg, evg[:900])
    check("PAGE-EV: empty states - reading, and an empty log",
          "reading the diag log" in ev.get("empty", {}).get("evs", "")
          and "THE DIAGNOSTIC LOG IS EMPTY" in ev.get("none", {}).get(
              "evg", ""), repr((ev.get("empty"), ev.get("none")))[:400])
    ln = ev.get("lens") or [0, 0, 0]
    check("PAGE-EV-LOAD: after the first status, /diag?f=1 page by page "
          "(bytes counted, x-user-defined), then f=0; a minute later f=0 "
          "from where it got to; a smaller f=0 (rotated) reads both again "
          "from 0, the counts rebuilt, not added",
          ev.get("urls") == [
              "/diag?f=1&o=0", "/diag?f=1&o=5000", "/diag?f=0&o=0",
              "/diag?f=0&o=%d" % ln[1], "/diag?f=0&o=%d" % (ln[1] + ln[2]),
              "/diag?f=1&o=0", "/diag?f=1&o=16384", "/diag?f=0&o=0"]
          and "x-user-defined" in (ev.get("mime") or "")
          and ev.get("load1") == [False, True, 166, 0]
          and ev.get("load2") == [2, [ln[1] + ln[2], ln[0]]]
          and ev.get("rot") == [168, 2, True, False]
          and "81<em>% of minutes" in ev.get("kpi1", "")
          and "GPS reset loop" in ev.get("grp1", ""),
          repr({k: ev.get(k) for k in ("urls", "load1", "load2", "rot",
                                       "lens", "mime")}))
    check("PAGE-EV-LOAD: no diag.1 (a 404) goes on to diag.log; a UTF-8 "
          "SSID arrives one char a byte, is shown decoded, the offset in "
          "bytes",
          ev.get("no404") == ["/diag?f=1&o=0", "/diag?f=0&o=0"]
          and (ev.get("utf") or [0, 1])[0] == (ev.get("utf") or [0, 1])[1]
          and (ev.get("utf") or [0, 0, False])[2] is True,
          repr((ev.get("no404"), ev.get("utf"))))
    # PAGE-EV-KINDS: one line of every diag format gps_logger.py writes;
    # the important ones in their own kind, Other only for the clock and
    # the heap notes. Every format in the source must have a sample here
    ev_samples = [
        ("gps reset loop 5 restarts in 6 s", "gloop"),
        ("gps reset loop 3 more windows", "gloop"),
        ("gps reset loop 4 banners, no NMEA", "gloop"),
        ("gps u-blox M10, ROM SPG 5.10 (7b202e), PROTVER=34.10", "grx"),
        ("gps rate 100 ms refused", "setup"),
        ("gps refused CFGMSG,0,7,5", "setup"),
        ("gps systems h135155", "grx"),
        ("gps refused CFGSYS,h135155", "setup"),
        ("gps setup lost: no GST", "setup"),
        ("gps pvt no NAV-PVT, NMEA this boot", "pvt"),
        ("gps setup not taken: no NAV-PVT (2 more)", "setup"),
        ("gps silent 60 s, no bytes from the receiver", "gsil"),
        ("gps back after 75 s", "gsil"),
        ("boot cause 1 prev_up 26731 mhz 160 mem 12912", "boot"),
        ("heap grew in connect to 66 KB", "other"),
        ("env SSID2: an SSID already set above, ignored", "env"),
        ("env WPA2_2 empty: SSID2 is tried as an open network", "env"),
        ("env FOO ignored (SSID, SSID2, SSID3 and WPA2, WPA2_2, WPA2_3 "
         "only)", "env"),
        ("wifi fail krumpli status 201 after 12000 ms (+2 more)", "wfail"),
        ("wifi back krumpli after 2 min 05 s, 3 failed tries, rssi -60",
         "wback"),
        ("wifi lost krumpli rssi -62", "wlost"),
        ("track write failing 20261102_100000.trk new 20261102_100100.trk",
         "track"),
        ("track create failing 20261102_100000.trk new x.trk", "track"),
        ("track write error 3 OSError(28) 20261102_100000.trk", "track"),
        ("time back 20261102T100500Z 20261102T100400Z", "tback"),
        ("stay left wifi aps 3 scans 6", "stay"),
        ("track full 20261102T1000-20261102T1005 drop 3", "full"),
        ("upload: /gpx/20261102_100000.trk cannot be deleted, parked as "
         ".err", "track"),
        ("upload ca err OSError(5)", "upl"),
        ("clock set 2026-11-02T11:38:31Z", "other"),
        ("upload err html 403 (captive portal?)", "upl"),
        ("upload aborted after 120 s", "upl"),
        ("upload err OSError(113)", "upl"),
        ("reboot tls memory 3", "reboot"),
        ("reboot low idf 40 KB 300 s", "reboot"),
        ("reboot guarded memory upload", "reboot"),
        ("upload API_KEY in .env: a space, control or non-ASCII character",
         "env"),
        ("upload GEO_URL in .env: not http(s)://host[:port][/path]", "env"),
        ("upload low idf 40 KB", "low"),
        ("upload low mem 9000 B", "low"),
        ("ota a3f2271 page /web.html.gz not put up: OSError(28)", "otax"),
        ("ota: manifest fields refused", "otax"),
        ("ota build 1793548800: manifest hmac does not match", "otax"),
        ("ota a3f2271: download 404, sha256 mismatch", "otax"),
        ("ota a3f2271 installed (was 2e8bc6e), rebooting", "ota"),
        ("ota a3f2271 booted, checking", "ota"),
        ("ota a3f2271 set to boot, not run yet: rebooting into it", "ota"),
        ("ota a3f2271 crashed, rolled back by the bootloader: bad (running "
         "2e8bc6e)", "ota"),
        ("ota a3f2271 rolled back after no fix, 3 times: bad (running "
         "2e8bc6e)", "ota"),
        ("ota a3f2271 rolled back after no upload, tried again in 60 min "
         "(running 2e8bc6e)", "ota"),
        ("ota a3f2271 ok", "ota"),
        ("ota a3f2271 no fix in 900 s, rolling back, bad", "ota"),
        ("ota: uploads stuck 6 h (err 503), checking anyway", "otax"),
        ("ota error OSError(-202)", "otax"),
        ("web error KeyError('y')", "bug"),
        ("bug upload ValueError('x')", "bug"),
        ("bug tft OSError(5)", "bug"),
        ("crash up 3600 MemoryError()", "crash"),
        ("up 60 utc - nmea 0/0 gpsboots 0 loops 0 sats 0/None fix "
         "No_data! hdop None points 0 sent 0 wifi off pulses 0 mem 27000 "
         "idf None/None heap 124", "status")]
    with open(os.path.join(ROOT, "gps_logger.py")) as f:
        ev_src = f.read()
    ev_fmts = re.findall(r'_wifi_diag\(\s*"\w+",\s*"([^"]*)"', ev_src)
    for m_ in re.finditer(r'(_diag_write|_ota_say|_env_warn|_loop_diag|'
                          r'_bug_once)\((\s*now,)?\s*"([^"]*)"', ev_src):
        ev_fmts.append(m_.group(3))

    def _ev_pat(fmt):
        out = ""
        for part in re.split(r"(%[-.\d]*[dsrf])", fmt):
            if not part:
                continue
            if part.startswith("%"):
                out += r"-?\d+" if part.endswith("d") else r".+?"
            else:
                out += re.escape(part)
        return out

    ev_nomatch = [f_ for f_ in ev_fmts if not any(
        re.match(_ev_pat(f_), l_) for l_, _ in ev_samples)]
    ev_lf = os.path.join(tempfile.mkdtemp(prefix="evk-"), "lines.json")
    with open(ev_lf, "w") as f:
        json.dump([l_ for l_, _ in ev_samples], f)
    r_ = subprocess.run([node, os.path.join(here, "web_page_test.js"),
                         os.path.join(here, "..", "web.html")],
                        input=page_json, capture_output=True, text=True,
                        env=dict(os.environ, EV_LINES=ev_lf))
    try:
        ev_kinds = json.loads(r_.stdout).get("kinds")
    except ValueError:
        ev_kinds = r_.stdout[-300:] + r_.stderr[-300:]
    ev_bad = [(l_, k_, g_) for (l_, k_), g_ in zip(ev_samples, ev_kinds or [])
              if k_ != g_]
    check("PAGE-EV-KINDS: every diag format gps_logger.py writes has a "
          "sample (%d formats), each sample in its kind: crashes, bugs, "
          "upload errors, OTA, GPS silent / reset loop / setup / pvt, WiFi, "
          "track and clock, reboots, boots, receiver; Other only the clock "
          "set and heap notes" % len(ev_fmts),
          len(ev_fmts) >= 45 and not ev_nomatch and isinstance(ev_kinds, list)
          and len(ev_kinds) == len(ev_samples) and not ev_bad,
          repr((len(ev_fmts), ev_nomatch, ev_bad,
                ev_kinds if not isinstance(ev_kinds, list) else "")))
    # PAGE-WIFI: an arc icon (0-3 arcs lit) and the quality in per cent,
    # 2 x (RSSI + 100) clamped; amber below -67 dBm, red below -80; no words
    check("PAGE-WIFI: arcs, quality % and tone from RSSI",
          v.get("wq") == [[3, 100, ""], [3, 80, ""], [2, 74, ""], [2, 66, ""],
                          [1, 64, "warn"], [1, 40, "warn"], [0, 38, "bad"],
                          [0, 0, "bad"], [0, 0, "bad"], None],
          repr(v.get("wq")))
    check("PAGE-WIFI: Uplink cells - the network, the signal in % with a "
          "word by its tone and the dBm",
          "<small>Network</small><b class=\"\">&lt;x&gt;" in sy
          and '<small>Signal &middot; -75 dBm</small><b class="warn">50% weak'
              in sy, sy[sy.find("Network"):sy.find("Last upload")])
    # PAGE-NULL: south/west coordinates, Python None in diag lines, quotes
    check("PAGE-NULL: S/W coordinates as absolute values",
          v.get("coordSW", "").startswith("33.85678&deg; S  151.21530&deg; W")
          and v.get("coordNE", "") == "47.10000&deg; N  18.40000&deg; E",
          repr((v.get("coordSW"), v.get("coordNE"))))
    check("PAGE-NULL: an empty status.json shows - (no None/null/NaN)",
          "heap -/- KB" in v.get("nullSys", "")
          and '<small>Network</small><b class="">-</b>' in v.get("nullSys", "")
          and "GPS link" not in v.get("nullSys", "")
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
    # the track's own line (the white casing under it has the same d)
    def main_path(svg):
        m = re.search(r'<path d="([^"]*)"', svg)
        return '<svg width="1" height="1"><path d="' + (m.group(1) if m else "")
    p1c = hd.get("path1", "")
    p2c = hd.get("path", "")
    p1 = main_path(p1c)
    p2 = main_path(p2c) + '"/>' + p2c[p2c.find("<circle"):]
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
    check("PAGE-HIST: the track blue on a white casing (the same line twice)",
          p2c.count('<path class="c" d="') == 1 and p2c.count('<path d="') == 1
          and re.search(r'<path class="c" d="([^"]*)"', p2c).group(1)
          == re.search(r'<path d="([^"]*)"', p2c).group(1), repr(p2c[:120]))
    check("PAGE-HIST: the shown track's start and end marked on the map",
          p2.count('<circle class="s"') == 1 and p2.count('<circle class="e"') == 1,
          repr(p2[-160:]))
    check("PAGE-HIST: zoom fitted to the track (15-17), layer moves with the "
          "tiles, TODAY km in the HUD",
          15 <= (hd.get("zoom") or 0) <= 16 and hd.get("trkT")
          and hd.get("day") == "<em>TODAY </em>%.2f KM" % hd.get("km", -1)
          and 0.28 < hd.get("km", 0) < 0.32, repr((hd.get("zoom"),
                                                    hd.get("day"))))
    check("PAGE-HIST: no running hist.km (nothing reads it: the HUD and "
          "the timeline sum the trips)", hd.get("kmField") is False,
          repr(hd.get("kmField")))
    check("PAGE-HIST: a 20 km drive fits whole between the header and the "
          "telemetry strip (below zoom 13); without a fix the track's centre",
          v.get("feat", {}).get("fitLive") == [[True, True, True]] * 2,
          repr(v.get("feat", {}).get("fitLive")))
    check("PAGE-MAP: the now card on the band's half away from the dot; on "
          "a phone always at the top, the track fitted under it",
          v.get("feat", {}).get("cardSide") == ["now top", "now", "now top", "50px", 98],
          repr(v.get("feat", {}).get("cardSide")))
    nw = v.get("feat", {}).get("now", {})
    check("PAGE-NOW: Photon's answer read as street and number (a place's "
          "name first), district and city under it; nothing named: none",
          nw.get("addr") == {"st": "Fűtőház utca 16", "sub": "Víziváros, Székesfehérvár"}
          and nw.get("poi") == {"st": "Berényi út 72", "sub": "Videoton, Székesfehérvár"}
          and nw.get("street") == {"st": "Budai út", "sub": "Székesfehérvár"}
          and nw.get("none") is None, repr(nw))
    check("PAGE-NOW: here for since the track came within reach (2 h), 0 "
          "km/h standing (an old track too); moving ~36 km/h; none when "
          "the track ends elsewhere or on a past day",
          nw.get("still") == [117, 0] and nw.get("go") == [True, 36]
          and nw.get("away") is None and nw.get("past") is None
          and nw.get("stale") == 0, repr(nw))
    check("PAGE-NOW: a stray fix (under 2 min off) does not end the stay, "
          "6 min away does",
          nw.get("stray") == 117 and nw.get("left") == 59, repr(nw))
    check("PAGE-NOW: heading over the track's last 30 m (north 0, east 90), "
          "none on a shorter one",
          nw.get("headN") == 0 and nw.get("headE") == 90
          and nw.get("headNone") is None, repr(nw))
    cd = nw.get("card", {})
    check("PAGE-NOW: the card: address, district and city, HERE FOR / SPEED "
          "/ ACCURACY / LAST FIX, the coordinates under them (a phone: one "
          "line); without an address the coordinates as its title",
          cd.get("nst") == "Fűtőház utca 16" and cd.get("nsub") == "Víziváros, Székesfehérvár"
          and "HERE FOR<b>2 h 00 min</b>" in cd.get("nr", "")
          and "SPEED<b>0 km/h</b>" in cd.get("nr", "")
          and "ACCURACY<b>&plusmn;4 m</b>" in cd.get("nr", "")
          and 'class="fa">LAST FIX<b>2 s</b>' in cd.get("nr", "")
          and cd.get("ncoord") == "47.18000&deg; N"
          and nw.get("bare", {}).get("nst") == "47.18000&deg; N"
          and "HERE FOR" not in nw.get("bare", {}).get("nr", "")
          and "SPEED<b>&ndash;</b>" in nw.get("bare", {}).get("nr", "")
          and "&plusmn;5 m" in nw.get("bare", {}).get("nr", "")
          and cd.get("nm") == "here 2 h 00 min &middot; 0 km/h &middot; &plusmn;4 m"
          and nw.get("bare", {}).get("nm") == "&plusmn;5 m", repr(nw))
    check("PAGE-NOW: the address from Atlas' Photon; asked again only after "
          "25 m and 20 s, one at a time",
          (nw.get("url") or "").startswith("https://atlas.crusand.org/api/v1/photon/reverse?lat=47.18000&lon=18.42000")
          and nw.get("got") == "Fűtőház utca 16" and nw.get("asks") == [1, 1, 2, 2],
          repr(nw))
    check("PAGE-NOW: moving, a cone the way the track goes",
          nw.get("cone") == ["cone on", "rotate(0deg)"], repr(nw.get("cone")))
    check("PAGE-MAP: the free band is measured (header bottom, strip top), "
          "defaults without a layout",
          v.get("feat", {}).get("band") == [[44, 204], [70, 250]],
          repr(v.get("feat", {}).get("band")))
    ft = v.get("feat", {})
    check("PAGE-LINK: GPS link tone by seconds since the last frame, B/s "
          "between polls, junk vs nothing, none on an old firmware",
          ft.get("linkOk") == {"tone": "", "rate": 400, "silent": 0,
                               "junk": False}
          and ft.get("linkWarn") == "warn"
          and ft.get("linkJunk", {}).get("silent") == 42
          and ft.get("linkJunk", {}).get("junk") is True
          and ft.get("linkDead") == {"tone": "bad", "rate": None,
                                     "silent": 95, "junk": False}
          and ft.get("linkOld") is None, repr(ft)[:600])
    check("PAGE-LINK: red banner only while silent; Systems GPS LINK line",
          ft.get("glOk") == "" and ft.get("glDead", "").startswith(
              "GPS SILENT &middot; 42 S<em>nothing from the receiver")
          and ft.get("alert", "").startswith("GPS SILENT &middot; 1 MIN 00 S")
          and '<div class="sx-r "><span><i></i>GPS link <em>400 B/s &middot; '
              'frame 0 s ago' in ft.get("sysOk", "")
          and "48.8 KB read</em>" in ft.get("sysOk", "")
          and '<div class="sx-r bad"><span><i></i>GPS link <em>frame 42 s '
              'ago' in ft.get("sysDead", "")
          and "GPS link" not in ft.get("sysNone", "-"),
          repr((ft.get("glDead"), ft.get("sysOk", "")[-400:])))
    check("PAGE-UPLOAD: LAST UPLOAD line; amber (and the strip) when points "
          "wait over 30 min",
          '<small>Last upload</small><b class="">30 s ago</b>'
          in ft.get("sysOk", "") and "<small>Waiting</small><b>0<em>pts"
          in ft.get("sysOk", "")
          and '<small>Last upload</small><b class="warn">1 h 06 min ago</b>'
              in ft.get("sysDead", "")
          and '<small>Waiting</small><b class="warn">50<em>pts'
              in ft.get("sysDead", "")
          and '<div class="warn"><small>UPLINK</small><b>50<em>pts' in
              ft.get("tmDead", "")
          and "<small>Last upload</small><b class=\"\">-</b>" in ft.get("sysNone", "-"),
          repr(ft.get("sysDead", "")[-500:]))
    pd = ft.get("pastDays") or ["?"]

    def _local_days(utc_days):
        # each UTC day: the local days of its first and last millisecond
        # (the page's dayList), today and later left out, today last
        today = time.strftime("%Y%m%d", time.localtime())
        out = set()
        for s in utc_days:
            t = calendar.timegm((int(s[:4]), int(s[4:6]), int(s[6:]), 0, 0, 0))
            for x in (t, t + 86399.999):
                d = time.strftime("%Y%m%d", time.localtime(x))
                if d < today:
                    out.add(d)
        return sorted(out) + [today]
    check("PAGE-DAYS: the days oldest first (bad, duplicate and today's "
          "skipped, today last); the arrow steps to a past local day, read "
          "once from its UTC files, drawn without a fix; and back to today",
          ft.get("days") == _local_days(["20261029", "20261030",
                                          ft.get("today")])
          and "20261030" in pd and len(pd) in (1, 2)
          and ft.get("pastEnd") == 86400000
          and ft.get("nav0") == ["TODAY", False, True]
          and ft.get("pickUrls") == ["/hist?d=%s&o=0" % pd[0]]
          and ft.get("pickPts") == 5 and ft.get("pickDone") is True
          and ft.get("pickMap") == "map nofix" and ft.get("pickTrk") is True
          and ft.get("pickDay") == "<em>2026-10-30 </em>4.42 KM"
          and ft.get("nav1") == ["2026-10-30", False, False]
          and ft.get("backToday") is True and ft.get("pts") == [
              [3600000, 120.5], [3601000, 121], [3602000, None]],
          repr({k: ft.get(k) for k in ("days", "pastDays", "pickUrls",
                                       "pickPts", "pickDay", "nav0",
                                       "nav1")}))
    check("PAGE-HIST: a track uploaded after a newer one is drawn in time "
          "order; no km from the jump back",
          ft.get("late") == [[3600000, 3601000, 7200000, 7201000], 221],
          repr(ft.get("late")))
    check("PAGE-TIMELINE: trips split at stops over 5 min, a few standing "
          "fixes folded into the stop; trip rows (km, time, km/h) and stop "
          "rows; the picked trip marked",
          ft.get("trips") == [[0, 20, 480, 500, 1.2], [24, 34, 660, 670, 0.5]]
          and ft.get("timeline") == (
              '<div class="tr go" onclick="hlTrip(0)"><time>08:00</time><i>'
              '</i><span><b>1.20 km</b> &middot; 20 min &middot; 4 km/h'
              '</span></div><div class="tr st"><time>08:20</time><i></i>'
              '<span>stop 2 h 40 min</span></div><div class="tr go on" '
              'onclick="hlTrip(1)"><time>11:00</time><i></i><span><b>0.50 '
              'km</b> &middot; 10 min &middot; 3 km/h</span></div><div '
              'class="tr end"><time>11:10</time><i></i><span></span></div>')
          and ft.get("timelineNone") == [
              '<span class="hint">NO TRIP THIS DAY</span>',
              '<span class="hint">NO TRIP YET TODAY</span>'],
          repr((ft.get("trips"), ft.get("timeline"))))
    check("PAGE-TIMELINE: km, moving time (min, then h:mm) and trips on "
          "top; a trip clicked is drawn on the map, clicked again the day",
          ft.get("kpis") == "<div><b>1.70</b><em>km</em></div><div><b>30</b>"
              "<em>min moving</em></div><div><b>2</b><em>trips</em></div>"
          and "<b>1:15</b><em>h moving</em>" in ft.get("kpisLong", "")
          and "<em>trip</em>" in ft.get("kpisLong", "")
          and "<b>1</b><em>trip</em>" in ft.get("pickKpi", "")
          and ft.get("hlMap") == 2 and ft.get("hlLine") is True
          and ft.get("hlOff") == 1,
          repr((ft.get("kpis"), ft.get("kpisLong"), ft.get("hlMap"))))
    check("REV-4: today is the local date (Budapest 00:30 = Nov 2), Nov 1 "
          "a past day",
          ft.get("tz") == [-60, ["20261031", "20261101", "20261102"]],
          repr(ft.get("tz")))
    check("REV-10: a deleted shown day keeps both arrows (to its neighbours)",
          ft.get("pruned") == [[False, False, True],
                               ["20261028", "20261030"]],
          repr(ft.get("pruned")))
    check("R2-PAGEERR: a bug drawing a box keeps LIVE and says so (not "
          "OFFLINE)",
          ft.get("pageErr", ["", ""])[0] == ""
          and ft.get("pageErr", ["", ""])[1].startswith(
              "LIVE &middot; PAGE ERROR boom"), repr(ft.get("pageErr")))
    check("WEBR-1: a UTC day file offers both local days it covers (UTC+1: "
          "the after-midnight part of the next day; UTC-5: the evening "
          "before); picking that day reads the file, its points shown",
          ft.get("wr1") == [["20261031", "20261101", "20261103"],
                            ["20261031", "20261101"], 2]
          and ft.get("wr1w") == ["20261030", "20261031", "20261103"],
          repr((ft.get("wr1"), ft.get("wr1w"))))
    check("WEBR-3: a past day with one point is centred on it (it was "
          "~58 000 px off the map); today with one point keeps the fix",
          ft.get("wr3") == [0, 0, 0, 0], repr(ft.get("wr3")))
    check("WEBR-4: /hist offsets count bytes (x-user-defined: one char a "
          "byte), so a non-ASCII byte neither shifts the next o= nor makes "
          "a full reply look short",
          ft.get("wr4") == [80, 80, True, 2], repr(ft.get("wr4")))
    check("HIST-NEXT: a .trb day is asked again from X-Hist-Next (not "
          "from o + the bytes received) while short of X-Hist-End, a minute "
          "later from there; a reply that does not move on is not asked "
          "again (no header: the bytes counted, WEBR-4)",
          ft.get("hnext") == [["/hist?d=D&o=0", "/hist?d=D&o=500",
                               "/hist?d=D&o=900", "/hist?d=D&o=900"],
                              3, 4, False], repr(ft.get("hnext")))
    check("WEBR-7: a poll like the last one rewrites no box (innerHTML comes "
          "back normalised, &middot; as U+00B7)",
          ft.get("wr7") == [[], True], repr(ft.get("wr7")))
    check("WEBR-8: rssi, temp, cno, elev and azim from the JSON reach the "
          "HTML coerced or escaped", ft.get("wr8") == [], repr(ft.get("wr8")))
    check("TELE-FROZEN: status lines that repeat a stale GPS time go on "
          "with up: a 3 min drop-out spans 3 min", ft.get("frozen") ==
          [300000, [[180000, 3]]], repr(ft.get("frozen")))
    check("WEBR-2: a 500 from the board shows its error, not NO ANSWER",
          ft.get("wr2") == ["off", "BOARD ERROR &middot; "
                                   "TypeError(&#39;row bug&#39;)"],
          repr(ft.get("wr2")))
    gx = ft.get("gpx", "")
    check("PAGE-GPX: GPX 1.1, a segment per 5 min gap, ele when known, "
          "escaped name",
          gx.startswith('<?xml version="1.0" encoding="UTF-8"?>\n<gpx '
                        'version="1.1"')
          and "<name>GPS &lt;day&gt;</name>" in gx
          and gx.count("<trkseg>") == 2
          and '<trkpt lat="47.1" lon="18.4"><ele>120.5</ele><time>'
              '2026-10-30T08:00:00Z</time></trkpt>' in gx
          and '<trkpt lat="47.2" lon="18.5"><time>' in gx
          and gx.endswith("</trkseg>\n</trk>\n</gpx>\n"), gx[:400])
    check("PAGE: satellite table and map tile maths",
          "<td>SBAS 5</td><td>-</td><td>-</td><td>-</td>" in v.get("svt", "")
          and v.get("svh") == "Satellite table &middot; 3"
          and v.get("tile", {}).get("x") == 72235
          and v.get("tile", {}).get("y") == 46048, repr(v.get("tile")))
with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                       "web.html")) as f:
    page_src = f.read()
check("PAGE-STATIC-GONE: no Static test panel, code, CSS or browser "
      "storage on the page",
      not any(w_ in page_src for w_ in (
          "Static test", "statView", "statAdd", "sstat", "scat", "STAT_",
          "gps-static", "localStorage", "CEP50")), "")
# (the map: OSM-carto tiles through the homelab Atlas proxy, as Dawarich
# uses: one cache, Geoapify's key stays on CT 146; OSM's own tile servers
# ask apps not to use them for this)
check("PAGE: polls status.json at the board's refresh_s, no reload; Atlas "
      "map tiles, credited",
      "/status.json" in page_src and "refresh_s" in page_src
      and 'http-equiv="refresh"' not in page_src
      and "tile.openstreetmap.org" not in page_src
      and "https://atlas.crusand.org/tiles/geoapify/" in page_src
      and "OpenStreetMap</a>" in page_src and "Geoapify</a>" in page_src
      and '<link rel="icon" href="data:image/svg+xml,' in page_src
      and "el.offsetWidth / 512" in page_src and "window.onresize" in page_src
      and 'name="viewport"' in page_src)

# PAGE-LAYOUT: the page in headless Chrome at phone and tablet width, a
# status.json with long values, offline with a day's km (the HUD's longest
# state): no horizontal scrolling, the HUD items inside the screen and not
# over each other. Not in CI (no Chrome there): SKIP.
chrome = shutil.which("google-chrome") or shutil.which("chromium")
if chrome is None:
    print("SKIP  PAGE-LAYOUT: google-chrome not installed")
else:
    import functools
    import http.server
    import threading
    lay_d = tempfile.mkdtemp(prefix="page-")
    lay_j = dict(jd3)
    lay_j.update(
        diag=["up 600 utc 20261102T070000Z nmea 9/1 gpsboots 0 loops 0 sats "
              "14/9 fix 3D_fix hdop None points 512 sent 400 wifi connected "
              "pulses 3 mem 27000 idf 157/120 heap 124 ubx 900/0 hacc 2.1",
              "boot cause 1 prev_up 700 mhz 80",
              "upload err OSError 113 ECONNABORTED while posting the batch"],
        satellites=[{"name": "G12", "system": "GPS", "svid": 12, "cno": 38,
                     "elev": 45, "azim": 120, "used": True,
                     "unhealthy": False}],
        receiver="u-blox M10, ROM SPG 5.10", rx_bytes=123456, rx_age_s=0,
        good_age_s=0, gps_boots=2, points=123456, left=12345, files=12,
        sent=98765, upload="err OSError 113", upload_err=3,
        upload_age_s=4000, uptime=93784, prev_run="11 min 40 s",
        prev_end="2026-11-01T18:00:00", wifi="connected "
        "MyVeryLongHomeNetworkName_5G", rssi=-71, pulses=3, fw="d2c6db9",
        fw_build=1793520000, ota="current", ubx="900 / 0", flash_kb=2600,
        days=["20261030", "20261031"])
    # the probe: offline 12 min with a day's km, as stale() and trackPaint()
    # write them, then the page's and the HUD items' boxes
    lay_probe = (
        '<script>setTimeout(function () {'
        'last = Date.now() - 725000; stale();'
        'document.getElementById("day").innerHTML = "<em>TODAY </em>23.45 KM";'
        'var r = [], hs = document.querySelector(".hud").children;'
        'for (var i = 0; i < hs.length; i++) { var b = hs[i].getBoundingClientRect();'
        '  if (b.width) r.push([b.left, b.right, b.top, b.bottom]); }'
        'var o = document.createElement("pre"); o.id = "probe";'
        'o.textContent = JSON.stringify({sw: document.scrollingElement.scrollWidth,'
        '  iw: innerWidth, hud: r});'
        'document.body.appendChild(o); }, 3000)</script></body>')
    with open(os.path.join(lay_d, "index.html"), "w") as f:
        f.write(page_src.replace("</body>", lay_probe))
    with open(os.path.join(lay_d, "status.json"), "w") as f:
        json.dump(lay_j, f)
    # /diag?f=&o= (the server ignores the query: both files this one, no
    # X-Diag-Size): a day of status lines and long events for the Event log
    with open(os.path.join(lay_d, "diag"), "w") as f:
        for k in range(1440):
            f.write("up %d utc 20261102T%02d%02d00Z nmea 1/0 gpsboots 0 "
                    "loops 0 sats %d/%d fix 3D_fix hdop 1.2 points 0 sent 0 "
                    "wifi connected pulses 0 mem 17000 idf 221/208 heap 62\n"
                    % (60 * k, k // 60, k % 60, 8 + k % 9, 4 + k % 5))
            if k % 97 == 5:
                f.write("wifi fail MyVeryLongHomeNetworkName_5G status 201 "
                        "after 12000 ms (+5 more)\nupload err OSError 113 "
                        "ECONNABORTED while posting the batch\n")
    # headless Chrome keeps a window 500 px wide at least: the page runs in
    # a frame of the width (its viewport, media queries included)
    for w in (375, 800):
        with open(os.path.join(lay_d, "w%d.html" % w), "w") as f:
            f.write('<!doctype html><body style="margin:0"><iframe src="/" '
                    'style="width:%dpx;height:900px;border:0"></iframe>'
                    '<script>setTimeout(function () {'
                    'var p = document.querySelector("iframe").contentDocument'
                    '.getElementById("probe"), o = document.createElement("pre");'
                    'o.id = "probe"; o.textContent = p ? p.textContent : "none";'
                    'document.body.appendChild(o); }, 4000)</script></body>'
                    % w)

    class _LayH(http.server.SimpleHTTPRequestHandler):
        def log_message(self, *a):
            pass

    lay_srv = http.server.ThreadingHTTPServer(
        ("127.0.0.1", 0), functools.partial(_LayH, directory=lay_d))
    threading.Thread(target=lay_srv.serve_forever, daemon=True).start()
    lay = {}
    try:
        for w in (375, 800):
            r = subprocess.run(
                [chrome, "--headless=new", "--no-sandbox", "--disable-gpu",
                 # no fonts or tiles from the internet: the same in any lab
                 "--host-resolver-rules=MAP * ~NOTFOUND, EXCLUDE 127.0.0.1",
                 "--window-size=1000,1000", "--virtual-time-budget=6000",
                 "--dump-dom", "http://127.0.0.1:%d/w%d.html"
                 % (lay_srv.server_port, w)],
                capture_output=True, text=True, timeout=90)
            i = r.stdout.find('<pre id="probe">')
            try:
                lay[w] = json.loads(r.stdout[i + 16:r.stdout.index("</pre>", i)]
                                    .replace("&quot;", '"'))
            except ValueError:
                lay[w] = {"error": r.stdout[-300:] + r.stderr[-300:]}
    finally:
        lay_srv.shutdown()
        shutil.rmtree(lay_d, ignore_errors=True)

    def _lay_ok(m):
        h = m.get("hud", [])
        # (one line each: a squeezed item breaks its words over lines)
        return (m.get("iw") and m.get("sw", 1e9) <= m["iw"] and len(h) >= 3
                and all(b[1] <= m["iw"] + 0.5 and b[3] - b[2] <= 24
                        for b in h)
                and not any(a[0] < b[1] - 0.5 and b[0] < a[1] - 0.5
                            and a[2] < b[3] - 0.5 and b[2] < a[3] - 0.5
                            for k, a in enumerate(h) for b in h[k + 1:]))

    check("PAGE-LAYOUT: no horizontal scroll at 375 and 800 px "
          "(scrollWidth <= innerWidth), long Systems values included",
          all(lay.get(w, {}).get("iw") and lay[w].get("sw", 1e9)
              <= lay[w]["iw"] for w in (375, 800)), repr(lay)[:600])
    check("PAGE-LAYOUT: offline with a day's km the HUD fits the screen, "
          "each item on one line, none over another (375 and 800 px)",
          all(_lay_ok(lay.get(w, {})) for w in (375, 800)), repr(lay)[:600])


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

import inspect
# GPS-PINS-ENV: .env GPS_TX / GPS_RX move the GPS UART (a second receiver
# on the C6's old 4/5 wiring); pins the board or chip needs, both the same,
# or a UART that refuses keep the default
class _UartSpy(object):
    def __init__(self, fail=False):
        self.calls, self.fail = [], fail
    def init(self, **k):
        if self.fail:
            raise ValueError("bad pin")
        self.calls.append(k)
_gp_saved = (gl.uart, gl.GPS_TX, gl.GPS_RX, gl.IS_C6)
gp = []
try:
    gl.IS_C6 = True
    for env, fail in (({"GPS_TX": "4", "GPS_RX": "5 # old wiring"}, False),
                      ({"GPS_TX": "12", "GPS_RX": "16"}, False),
                      ({"GPS_TX": "4", "GPS_RX": "18"}, False),
                      ({"GPS_TX": "x", "GPS_RX": "9"}, False),
                      ({"GPS_TX": "2", "GPS_RX": "3"}, True),
                      ({}, False)):
        gl.GPS_TX, gl.GPS_RX = 18, 19
        gl.uart = _UartSpy(fail)
        gl._gps_pins_env(env)
        gp.append((gl.GPS_TX, gl.GPS_RX, list(gl.uart.calls)))
    gl.IS_C6 = False
    gl.GPS_TX, gl.GPS_RX = 17, 16
    gl.uart = _UartSpy()
    gl._gps_pins_env({"GPS_TX": "34", "GPS_RX": "12"})
    gp.append((gl.GPS_TX, gl.GPS_RX, list(gl.uart.calls)))
    gl._gps_pins_env({"GPS_TX": "27", "GPS_RX": "35"})
    gp.append((gl.GPS_TX, gl.GPS_RX, list(gl.uart.calls)))
finally:
    gl.uart, gl.GPS_TX, gl.GPS_RX, gl.IS_C6 = _gp_saved
check("GPS-PINS-ENV: GPS_TX/GPS_RX move the UART; console, USB, strapping "
      "8/9/12, same pin, input-only TX or a refused init keep the default",
      gp == [(4, 5, [{"baudrate": gl.BAUD, "tx": 4, "rx": 5}]),
             (18, 19, []),
             (4, 18, [{"baudrate": gl.BAUD, "tx": 4, "rx": 18}]),
             (18, 19, []),
             (18, 19, []),
             (18, 19, []),
             (17, 16, []),
             (27, 35, [{"baudrate": gl.BAUD, "tx": 27, "rx": 35}])]
      and "_gps_pins_env(env)" in inspect.getsource(gl.run),
      repr(gp))

# WEB-LINK: the page's GPS link - bytes read, seconds since the last whole
# NMEA sentence / UBX frame (a bad checksum is not one), None before any
def _nm(body):
    c = 0
    for ch in body.encode():
        c ^= ch
    return ("$%s*%02X\r\n" % (body, c)).encode()
gl.reset()
j0 = json.loads(gl._web_json())
_junk = b"\x00\xff garbage\r\n$GPRMC,1,2*00\r\n"
_uart_rx.extend(_junk)
gl.read_uart()
j1 = json.loads(gl._web_json())
junk_b, junk_good = gl.st["rx_b"], gl.st["good"]
_clock[0] += 4000
s1 = _nm("GPRMC,120500.00,V,,,,,,,171026,,,N")
_uart_rx.extend(s1)
gl.read_uart()
good_at = gl.st["good"]
_clock[0] += 7000
_uart_rx.extend(gl._ubx(0x0A, 0x04, bytes(40)))
gl.read_uart()
ubx_at = gl.st["good"]
_clock[0] += 12000
j2 = json.loads(gl._web_json())
check("WEB-LINK: rx_bytes counts every byte, good_age_s only whole frames",
      j0["rx_bytes"] == 0 and j0["rx_age_s"] is None
      and j0["good_age_s"] is None
      and junk_b == len(_junk) and junk_good is None and j1["good_age_s"] is None
      and j1["rx_age_s"] == 0 and good_at is not None
      and ubx_at == good_at + 7000
      and j2["rx_bytes"] == len(_junk) + len(s1) + 48
      and j2["good_age_s"] == 12 and j2["rx_age_s"] == 12,
      repr((j0.get("rx_bytes"), junk_b, junk_good, j1.get("good_age_s"),
            good_at, ubx_at, j2.get("rx_bytes"), j2.get("good_age_s"))))

# WEB-POS6: the position at the track line's precision for the static test:
# NAV-PVT's integer 1e-7 degrees exactly, NMEA's float to 6 decimals
gl.st.update(pos_at=_clock[0], valid=True, lat=47.1853612, lon=18.4273049, lat_i=471853612,
             lon_i=184273049)
p_pvt = json.loads(gl._web_json())
gl.st.update(lat_i=None, lon_i=None)
p_nm = json.loads(gl._web_json())
gl.st["valid"] = False
p_none = json.loads(gl._web_json())
check("WEB-POS6: pos6 from lat_i/lon_i (PVT), %.6f (NMEA), None w/o fix",
      p_pvt["pos6"] == "47.185361, 18.427305"
      and p_nm["pos6"] == "47.185361, 18.427305"
      and p_pvt["position"] == "47.18536, 18.42730"
      and p_none["pos6"] is None,
      repr((p_pvt.get("pos6"), p_nm.get("pos6"), p_none.get("pos6"))))
gl.reset()

# WEB-DAYS: the day history's days for the page's picker, oldest first,
# only YYYYMMDD.trb and (older firmware's) YYYYMMDD.trk, a day once,
# listed at most every 30 s
_hd_dir = tempfile.mkdtemp()
_hd_saved = gl.HIST_DIR
gl.HIST_DIR = _hd_dir
gl._hd[1] = None
for n in ("20261031.trk", "20261029.trb", "x.trk", "2026103.trb",
          "20261030.trk.tmp", "20261028.trb.tmp", "20261030.trk",
          "20261030.trb"):
    open(os.path.join(_hd_dir, n), "w").close()
days1 = gl._hist_days()
open(os.path.join(_hd_dir, "20261101.trb"), "w").close()
_clock[0] += 10000
days2 = gl._hist_days()
_clock[0] += 25000
days3 = json.loads(gl._web_json())["days"]
gl.HIST_DIR = os.path.join(_hd_dir, "missing")
gl._hd[1] = None
days4 = gl._hist_days()
gl.HIST_DIR = _hd_saved
gl._hd[1] = None
shutil.rmtree(_hd_dir, ignore_errors=True)
check("WEB-DAYS: sorted YYYYMMDD days, cached 30 s, [] without the folder",
      days1 == ["20261029", "20261030", "20261031"] and days2 == days1
      and days3 == days1 + ["20261101"] and days4 == [],
      repr((days1, days2, days3, days4)))

# WEB-UPLOAD-AGE: seconds since the last accepted batch, None before one
# (and after a rejected one only)
_ua_dir = tempfile.mkdtemp()
online(_ua_dir)
make_trk(os.path.join(_ua_dir, "20261031_120000.trk"), 3)
ua0 = json.loads(gl._web_json())["upload_age_s"]
_post_code[0] = 422
pump()
ua1 = gl.up["ok_at"]
online(_ua_dir)
make_trk(os.path.join(_ua_dir, "20261031_130000.trk"), 3)
pump()
_clock[0] += 61000
ua2 = json.loads(gl._web_json())["upload_age_s"]
shutil.rmtree(_ua_dir, ignore_errors=True)
check("WEB-UPLOAD-AGE: upload_age_s None until a 2xx, then its age",
      ua0 is None and ua1 is None and ua2 == 61 and gl.up["sent"] == 3,
      repr((ua0, ua1, ua2, gl.up.get("sent"))))


# C6-UART: the C6 reads the GPS on UART1 GPIO18 (TX) / GPIO19 (RX), not the
# strapping pins 4/5 (the GPS is soldered to 18/19 now); the ESP32 keeps 16/17
_uart_kw = []
_UART0 = sys.modules["machine"].UART
class _UARTSpy(_UART0):
    def __init__(self, *a, **k):
        _uart_kw.append((a, k))
_saved_machine = getattr(sys.implementation, "_machine", "")
sys.modules["machine"].UART = _UARTSpy
try:
    sys.implementation._machine = "Generic ESP32C6 module with ESP32C6"
    c6 = _reload_logger()
    c6_pins = (c6.IS_C6, c6.GPS_UART, c6.GPS_TX, c6.GPS_RX, _uart_kw[-1])
    sys.implementation._machine = "Generic ESP32 module with ESP32"
    wr = _reload_logger()
    wr_pins = (wr.IS_C6, wr.GPS_UART, wr.GPS_TX, wr.GPS_RX)
finally:
    sys.modules["machine"].UART = _UART0
    sys.implementation._machine = _saved_machine
    gl = _reload_logger()
check("C6-UART: C6 GPS on UART1 tx=18 rx=19, ESP32 on UART2 tx=17 rx=16",
      c6_pins[:4] == (True, 1, 18, 19)
      and c6_pins[4][0][:1] == (1,) and c6_pins[4][1].get("tx") == 18
      and c6_pins[4][1].get("rx") == 19
      and wr_pins == (False, 2, 17, 16), "%r %r" % (c6_pins, wr_pins))

# --- review fixes (REV-n: the n-th finding of the 2026-11-01 review) ------

_rev_dir = tempfile.mkdtemp()
gl.wifi_reset()
gl.LOG_DIR = _rev_dir
gl.log_reset()
gl.up_reset()

# REV-1: a silent receiver keeps its last second's valid flag; the page
# got that position as live (and the static test recorded it)
gl.reset()
gl.st.update(rx=_clock[0], pos_at=_clock[0], valid=True, fixtype=3,
             lat=47.1, lon=18.4)
r1a = json.loads(gl._web_json())
_clock[0] += gl.NO_DATA + 1000
r1b = json.loads(gl._web_json())
gl.reset()
check("REV-1: no position once the last second is older than NO_DATA",
      r1a["position"] == "47.10000, 18.40000" and r1a["pos6"] is not None
      and r1b["fix"] == "No data!" and r1b["position"] is None
      and r1b["pos6"] is None, repr((r1a.get("position"), r1b.get("fix"),
                                     r1b.get("position"))))

# REV-2: the home link dropping (a router restart) during a home stay
# keeps the stay a home stay (at_home() reads sleep_home off-line)
class _LostW(object):
    def status(self, arg=None):
        if arg:
            raise OSError(1)
        return 0                         # STAT_IDLE: the link is gone
    def isconnected(self):
        return False
r2 = []
for active in (True, False):
    gl.wifi_reset()
    gl.log_reset()
    gl.wifi["nets"] = [{"ssid": "home", "pw": "x", "home": True,
                        "hidden": False, "bssid": None, "ip": None}]
    gl.wifi.update(wlan=_LostW(), state="connected", ssid="home", home=True,
                   last_check=_clock[0])
    gl.log["still"]["active"] = active
    gl.wifi_tick()
    r2.append((gl.wifi["state"], bool(gl.at_home())))
gl.wifi_reset()
gl.log_reset()
check("REV-2: link lost at home: a home stay stays at home, no stay is not",
      r2 == [("lost", True), ("lost", False)], repr(r2))

# REV-3: a track file whose writes keep failing (a bad block) is left after
# WFAIL_SWITCH failures; the batch goes on in a new file, the old uploads
_r3 = tempfile.mkdtemp()
gl.LOG_DIR = _r3
gl.log_reset()
gl.start_file("20261101_100000")
gl.log["batch"].extend(trk_line(47.1, 18.4, "2026-11-01T10:00:%02dZ" % i)
                       for i in range(3))
gl.flush()
f3 = gl.log["file"]
_open3 = builtins.open
def _bad_open(name, mode="r", *a, **k):
    if name == f3 and mode != "rb":
        raise OSError(5)
    return _open3(name, mode, *a, **k)
builtins.open = _bad_open
try:
    for r in range(getattr(gl, "WFAIL_SWITCH", 3)):
        gl.log["batch"].append(trk_line(47.2, 18.4,
                                        "2026-11-01T10:05:%02dZ" % r))
        gl.flush()
finally:
    builtins.open = _open3
f3b = gl.log["file"]
gl.flush()
new3 = pts_in(f3b) if f3b != f3 and os.path.exists(f3b) else None
pend3 = [os.path.basename(x) for x in gl.pending()]
check("REV-3: after WFAIL_SWITCH failed writes a new file takes the batch; "
      "the old one (3 points) is left to upload",
      f3b != f3 and new3 == 3 and pts_in(f3) == 3
      and os.path.basename(f3) in pend3 and not gl.log["batch"],
      repr((f3, f3b, new3, pend3)))
gl.LOG_DIR = _rev_dir
gl.log_reset()
shutil.rmtree(_r3, ignore_errors=True)

# REV-3b: a failing append that partly landed (half the batch, no footer)
# is cut back to the last good footer before the switch: the new file took
# the whole batch again, its first half uploaded twice (67 points, 60 new)
_r3 = tempfile.mkdtemp()
gl.LOG_DIR = _r3
gl.log_reset()
gl.start_file("20261101_100000")
gl.log["batch"].extend(trk_line(47.1, 18.4 + i * 1e-4,
                                "2026-11-01T10:00:%02dZ" % i)
                       for i in range(30))
gl.flush()
f3 = gl.log["file"]
gl.log["batch"].extend(trk_line(47.1, 18.4 + i * 1e-4,
                                "2026-11-01T10:01:%02dZ" % i)
                       for i in range(30))


class _HalfW(object):
    """A file whose longer writes land half and fail (ENOSPC)."""
    def __init__(self, f):
        self.f = f

    def seek(self, *a):
        return self.f.seek(*a)

    def tell(self):
        return self.f.tell()

    def read(self, *a):
        return self.f.read(*a)

    def write(self, data):
        if len(data) > 20:
            self.f.write(data[:len(data) // 2])
            raise OSError(28)
        return self.f.write(data)

    def close(self):
        self.f.close()


def _half_open(name, mode="r", *a, **k):
    f = _open3(name, mode, *a, **k)
    return _HalfW(f) if name == f3 and "+" in mode else f


def _trk_pts(path):
    out = []
    for line in open(path):
        if line == gl.FOOTER:
            break
        if line.count(",") == 4:
            out.append(line)
    return out


builtins.open = _half_open
try:
    for r in range(gl.WFAIL_SWITCH):
        gl.flush()
finally:
    builtins.open = _open3
f3b = gl.log["file"]
gl.flush()
p3 = [_trk_pts(f3), _trk_pts(f3b) if f3b != f3 else []]
all3 = p3[0] + p3[1]
check("REV-3b: a partly landed batch is not written twice across the "
      "WFAIL_SWITCH file change", f3b != f3 and len(p3[0]) == 30
      and len(all3) == 60 and len(set(all3)) == 60,
      "old=%d new=%d unique=%d" % (len(p3[0]), len(p3[1]), len(set(all3))))
# REV-3c: the footer cut runs after writes failed, often for want of heap:
# a MemoryError there is a failed write, not a crash (run() reset the board)
def _oom_open(name, mode="r", *a, **k):
    if "+" in mode:
        raise MemoryError
    return _open3(name, mode, *a, **k)


err0 = gl.log["err"]
builtins.open = _oom_open
try:
    try:
        gl._cut_to_footer(f3, 100, 200)
        cut_r = "ok"
    except MemoryError:
        cut_r = "MemoryError escaped"
finally:
    builtins.open = _open3
check("REV-3c: a MemoryError while cutting a failing file back to its "
      "footer is counted, not raised", cut_r == "ok"
      and gl.log["err"] == err0 + 1, cut_r)
gl.LOG_DIR = _rev_dir
gl.log_reset()
shutil.rmtree(_r3, ignore_errors=True)

# FLUSH-ATOMIC: nothing allocates between a write that landed and the
# batch letting go of its lines: a MemoryError there (the last line's time
# sliced for the segment queue) left the k lines in the batch, and the
# crash handler's flush appended them a second time
_r6 = tempfile.mkdtemp()
gl.LOG_DIR = _r6
gl.log_reset()
gl.start_file("20261101_100000")
_arm6 = [False, True]


class _ArmStr(str):
    """A track line whose slices fail once armed (a heap that is gone)."""
    def __getitem__(self, i):
        if _arm6[0] and isinstance(i, slice):
            raise MemoryError("heap")
        return str.__getitem__(self, i)


class _ArmW(object):
    """A file that arms _ArmStr when it closes: the write landed."""
    def __init__(self, f):
        self.f = f

    def seek(self, *a):
        return self.f.seek(*a)

    def tell(self):
        return self.f.tell()

    def read(self, *a):
        return self.f.read(*a)

    def write(self, data):
        return self.f.write(data)

    def close(self):
        self.f.close()
        _arm6[0] = _arm6[1]                # (the first write only)
        _arm6[1] = False


l6 = [trk_line(47.1, 18.4 + i * 1e-4, "2026-11-01T10:00:%02dZ" % i)
      for i in range(10)]
gl.log["batch"].extend(l6[:5])
gl.flush()
f6 = gl.log["file"]
gl._seg_break()
gl._record(("2026-11-01T10:00:05Z", _ArmStr(l6[5])), (47.1, 18.4))
gl.log["batch"].extend(_ArmStr(x) for x in l6[6:])


def _arm_open(name, mode="r", *a, **k):
    f = _open3(name, mode, *a, **k)
    return _ArmW(f) if name == f6 and "+" in mode else f


r6 = "ok"
builtins.open = _arm_open
try:
    try:
        gl.flush()
    except MemoryError:
        r6 = "MemoryError"
        _arm6[0] = False
        gl.flush()                         # (the crash handler's)
finally:
    builtins.open = _open3
    _arm6[0] = False
p6 = _trk_pts(f6)
check("FLUSH-ATOMIC: a MemoryError after a write landed does not write "
      "its lines twice", r6 == "ok" and len(p6) == 10
      and len(set(p6)) == 10 and not gl.log["batch"]
      and gl.log["new_seg"] is False,
      repr((r6, len(p6), len(set(p6)), len(gl.log["batch"]))))
gl.LOG_DIR = _rev_dir
gl.log_reset()
shutil.rmtree(_r6, ignore_errors=True)

# WFAIL-NAME: a create that keeps failing ("wb" refused for that name)
# goes on under a new name, one second on, after WFAIL_SWITCH tries (it
# retried the same name every FLUSH, nothing written ever); a diag line
_r7 = tempfile.mkdtemp()
gl.LOG_DIR = _r7
gl.log_reset()
diag_clear()
gl.start_file("20261101_100000")
f7 = gl.log["file"]
gl.log["batch"].extend(trk_line(47.1, 18.4 + i * 1e-4,
                                "2026-11-01T10:00:%02dZ" % i)
                       for i in range(3))


def _nowb_open(name, mode="r", *a, **k):
    if name == f7 and "w" in mode:
        raise OSError(5)
    return _open3(name, mode, *a, **k)


builtins.open = _nowb_open
try:
    for r in range(gl.WFAIL_SWITCH):
        gl.flush()
finally:
    builtins.open = _open3
f7b = gl.log["file"]
gl.flush()
d7 = [x for x in diag_lines() if x.startswith("track create failing")]
check("WFAIL-NAME: a create that keeps failing goes on one second on "
      "(20261101_100001), noted in the diag log",
      f7b != f7 and f7b.endswith("20261101_100001" + os.path.splitext(f7)[1])
      and os.path.exists(f7b) and pts_in(f7b) == 3 and not gl.log["batch"]
      and len(d7) == 1, repr((f7, f7b, d7)))
gl.LOG_DIR = _rev_dir
gl.log_reset()
diag_clear()
shutil.rmtree(_r7, ignore_errors=True)

# WERR-DIAG: track write errors (log["err"]) are in the diag log, the
# first at once, then one per WERR_EVERY (10 min): the count was shown
# nowhere
_r8w = tempfile.mkdtemp()
gl.LOG_DIR = _r8w
gl.log_reset()
diag_clear()
gl.start_file("20261101_120000")
f8w = gl.log["file"]


def _eio_open(name, mode="r", *a, **k):
    if name == f8w and mode != "rb":
        raise OSError(5)
    return _open3(name, mode, *a, **k)


n8w = []
builtins.open = _eio_open
try:
    for r in range(3):
        if r == 2:
            _clock[0] += 600000
        gl.log["wfails"] = 0               # (no WFAIL_SWITCH here)
        gl.log["batch"].append(trk_line(47.1, 18.4,
                                        "2026-11-01T12:00:%02dZ" % r))
        gl.flush()
        _clock[0] += gl.FLUSH
        n8w.append(len([x for x in diag_lines()
                        if x.startswith("track write error")]))
finally:
    builtins.open = _open3
check("WERR-DIAG: the first track write error goes to the diag log, then "
      "one per 10 min", n8w == [1, 1, 2] and gl.log["err"] == 3,
      repr((n8w, gl.log["err"], diag_lines()[-3:])))
gl.LOG_DIR = _rev_dir
gl.log_reset()
diag_clear()
shutil.rmtree(_r8w, ignore_errors=True)

# REV-6/7: a scan that raises MemoryError, or returns [] (v1.29 does on a
# failed esp_wifi_scan_start), is a failed scan: no crash, None
class _ScanW(object):
    def __init__(self, how):
        self.how = how
    def scan(self):
        if self.how == "mem":
            raise MemoryError("scan")
        return []
    def active(self, x=None):
        return True
gl.wifi_reset()
gl.wifi["nets"] = [{"ssid": "home", "pw": "x", "home": True,
                    "hidden": False, "bssid": None, "ip": None}]
r6 = []
for how in ("mem", "empty"):
    try:
        r6.append(gl._scan_known(_ScanW(how)))
    except MemoryError:
        r6.append("raised")
try:
    gl._pb_pulse_radio(_ScanW("mem"), True)
    r6.append("pulse ok")
except MemoryError:
    r6.append("pulse raised")
gl.wifi_reset()
check("REV-6/7: scan MemoryError and an empty scan read as a failed scan",
      r6 == [None, None, "pulse ok"], repr(r6))

# REV-8: the uploaded file that cannot be deleted keeps its .pos (it was
# dropped, and the whole file went up again and again)
_r8 = tempfile.mkdtemp()
p8 = os.path.join(_r8, "20261101_110000.trk")
make_trk(p8, 3)
gl.write_offset(p8, 40)
_rm8 = os.remove
def _no_rm(path, *a):
    if path == p8:
        raise OSError(1)
    return _rm8(path, *a)
os.remove = _no_rm
try:
    try:
        gl.commit_offset(p8, 3, 100, eof=True)
        r8 = "no error"
    except OSError:
        r8 = "raised"
finally:
    os.remove = _rm8
check("REV-8: a failed delete at EOF raises and keeps the .pos",
      r8 == "raised" and os.path.exists(p8 + ".pos"), repr(r8))
shutil.rmtree(_r8, ignore_errors=True)

# REV-9: a client dribbling its request head gets WEB_HEAD_WAIT, not the
# whole WEB_DEADLINE (no NMEA is read while recv waits); UART read between
class _Drip(object):
    def __init__(self):
        self.n = 0
    def settimeout(self, t):
        pass
    def recv(self, n):
        _clock[0] += 300
        self.n += 1
        return b"x"
_uart_rx.extend(b"$GPTXT,01,01,02,hello*00\r\n")
c9 = _Drip()
t9 = _clock[0]
gl._web_end[0] = time.ticks_add(_clock[0], gl.WEB_DEADLINE * 1000)
try:
    gl._web_read(c9)
    r9 = "returned"
except OSError:
    r9 = "timed out"
finally:
    end9 = gl._web_end[0]
    gl._web_end[0] = None
took9 = _clock[0] - t9
check("REV-9: a dribbling head is cut after WEB_HEAD_WAIT, the deadline "
      "restored, the UART read meanwhile",
      r9 == "timed out" and took9 <= 1300 and not _uart_rx
      and end9 == time.ticks_add(t9, gl.WEB_DEADLINE * 1000),
      repr((r9, took9, len(_uart_rx))))
del _uart_rx[:]

# REV-11: counters stay small ints (heap longs every loop after 12 days);
# "good" ages out with the other receiver stamps
gl._up[:] = [0, _clock[0], 0]
c11 = _clock[0]
for d in range(20):
    c11 += 86400000
    up11 = gl.uptime_s(c11)
small11 = gl._up[0] < (1 << 30) and len(gl._up) == 3 and 0 <= gl._up[2] < 1000
gl.reset()
gl.st["rx_b"] = 0x3FFFFFF0
_uart_rx.extend(b"$" * 32)
gl.read_uart()
rx11 = gl.st["rx_b"]
del _uart_rx[:]
gl.reset()
gl.st["good"] = _clock[0]
_clock[0] += gl.TICKS_CAP + 1000
gl._stamps_old(_clock[0])
good11 = gl.st["good"]
gl.reset()
gl._up[:] = [0, _clock[0], 0]
check("REV-11: uptime in seconds + ms (20 days), the byte count wraps at "
      "2^30, good cleared after TICKS_CAP",
      up11 == 20 * 86400 and small11 and rx11 == 16 and good11 is None,
      repr((up11, list(gl._up), rx11, good11)))

# REV-12: trk2gpx stops at the footer (bytes after it are left over from a
# longer older write)
g12 = trk2gpx.convert(["#TRK1\n", "47.1,18.4,1,1,20261026T100000Z\n",
                       "#END\n", "47.9,18.9,1,1,20261026T095959Z\n"], "x")
check("REV-12: trk2gpx ignores what follows #END",
      g12.count("<trkpt") == 1 and "47.9" not in g12, g12[-200:])

# REV-13: a receiver restart read during an acknowledged setup keeps the
# wait txt() set (a storm's GPS_STORM_WAIT), not GPS_RETRY again
gl.reset()
_ack13 = gl._ubx_ack
def _ack_restart(*a, **k):
    if gl.st["gps_boots"] == 0:
        gl.st["gps_boots"] += 1
        gl.st["setup_wait"] = gl.GPS_STORM_WAIT
    return True
gl._ubx_ack = _ack_restart
try:
    gl.gps_setup()
finally:
    gl._ubx_ack = _ack13
w13 = gl.st["setup_wait"]
gl.reset()
check("REV-13: a restart during the setup keeps the storm wait",
      w13 == gl.GPS_STORM_WAIT, repr(w13))

# REV-14: an interface that failed to come up at boot is made again after
# the backoff (it stayed down for good)
gl.wifi_reset()
gl.wifi["nets"] = [{"ssid": "home", "pw": "x", "home": True,
                    "hidden": False, "bssid": None, "ip": None}]
gl.wifi.update(state="error", wlan=None, last_check=_clock[0])
gl.wifi_tick()
early14 = gl.wifi["wlan"]
_clock[0] += gl.wifi["backoff"] + 1000
gl.wifi_tick()
made14 = gl.wifi["wlan"] is not None
gl.wifi_reset()
check("REV-14: WLAN made again after the backoff, not before",
      early14 is None and made14, repr((early14, made14)))

# REV-15: strapping pins refused as KEEPALIVE_PIN: C6 GPIO15, ESP32 GPIO12
saved15 = gl.IS_C6
try:
    gl.IS_C6 = True
    c15 = (gl._pin_taken(15), gl._pin_taken(2), gl._pin_taken(3))
    gl.IS_C6 = False
    e15 = (gl._pin_taken(12), gl._pin_taken(26), gl._pin_taken(13))
finally:
    gl.IS_C6 = saved15
check("REV-15: C6 15 and ESP32 12 refused, the defaults kept",
      c15 == (True, False, False) and e15 == (True, False, False),
      repr((c15, e15)))

# REV-16: an append after one cut short (half a line, no newline) starts
# on a new line: no ghost point glued from the two
_r16 = tempfile.mkdtemp()
_hd16 = gl.HIST_DIR
gl.HIST_DIR = _r16
with open(os.path.join(_r16, "20261101.trk"), "w") as f:
    f.write("47.100000,18.400000,1,2,1,20261101T100000Z\n47.4979")
gl._hist_add(["47.200000,18.500000,1,2,1,20261101T100001Z\n"], 1)
with open(os.path.join(_r16, "20261101.trk")) as f:
    h16 = f.read()
p16 = os.path.join(_r16, "20261101_100000.trk")
make_trk(p16, 2)
with open(p16 + ".bad", "w") as f:
    f.write("47.4979")
gl._append_bad(p16, 0, os.path.getsize(p16))
with open(p16 + ".bad") as f:
    b16 = f.read()
gl.HIST_DIR = _hd16
shutil.rmtree(_r16, ignore_errors=True)
check("REV-16: hist and .bad appends after a cut line start a new line",
      h16.endswith("\n47.4979\n47.200000,18.500000,1,2,1,20261101T100001Z\n")
      and b16.startswith("47.4979\n52.500000,13.400000,"),
      repr((h16[-80:], b16[:60])))

# WIFI-DIAG: a drop, the failed tries and the way back go to the diag log
# (only the console had them: a drop at home could not be explained from
# the page), one line of a kind per WIFI_DIAG_GAP, the rest counted
class _GoneW(object):
    def status(self, arg=None):
        if arg:
            raise OSError(1)
        return 0
class _BackW(object):
    def ifconfig(self):
        return ("192.168.1.60", "255.255.255.0", "192.168.1.1", "1.1.1.1")
    def status(self, arg=None):
        return -63 if arg == "rssi" else 3
_wd_saved = gl.DIAG
gl.DIAG = 60000
diag_clear()
gl.wifi_reset()
gl.log_reset()
gl.wifi["nets"] = [{"ssid": "home", "pw": "x", "home": True,
                    "hidden": False, "bssid": None, "ip": None}]
gl.wifi.update(wlan=_GoneW(), state="connected", ssid="home", home=True,
               rssi=-61, last_check=_clock[0])
gl.wifi_tick()
gl._mark_failed("home", "status 202 after 7000 ms")
_clock[0] += 1000
gl._mark_failed("home", "status 202 after 7000 ms")
_clock[0] += gl.WIFI_DIAG_GAP
gl._mark_failed("home", "status 201 after 300 ms")
_clock[0] += 125000
gl._wifi_got_ip(_BackW())
wd = [l for l in diag_lines() if l.startswith("wifi ")]
gl.DIAG = _wd_saved
diag_clear()
gl.wifi_reset()
check("WIFI-DIAG: lost (ssid, rssi), fails rate-limited with the count, "
      "back (after, tries, rssi) in the diag log",
      wd == ["wifi lost home rssi -61",
             "wifi fail home status 202 after 7000 ms",
             "wifi fail home status 201 after 300 ms (+1 more)",
             "wifi back home after 7 min 06 s, 3 failed tries, rssi -63"],
      repr(wd))

# --- OTA: manifest, HMAC, slot write, rollback (OTA-*) ----------------------
import hmac as _hmac_mod
import hashlib as _hl
for key, msg in ((b"k", b"m"), (b"x" * 64, b"abc"), (b"y" * 100, b"1|2|3")):
    want = _hmac_mod.new(key, msg, _hl.sha256).hexdigest()
    if gl._hmac_hex(key, msg) != want:
        break
check("OTA-HMAC: HMAC-SHA256 as Python's hmac (short, 64-byte, long key)",
      gl._hmac_hex(key, msg) == want, key[:4])


class _FakePart(object):
    def __init__(self, name, size):
        self.name, self.mem = name, bytearray(b"\0" * size)
        self.boot = False
        self.writes = 0
    def info(self):
        return (0, 16, 0x10000, len(self.mem), self.name, False)
    def writeblocks(self, blk, buf):
        o = blk * 4096
        self.mem[o:o + len(buf)] = buf
        self.writes += 1
    def readblocks(self, blk, buf):
        o = blk * 4096
        buf[:] = self.mem[o:o + len(buf)]
    def set_boot(self):
        _ota_log.append(("boot", self.name))
        self.boot = True
    def get_next_update(self):
        return _slots[1] if self is _slots[0] else _slots[0]


_boot_part = [None]                     # Partition(BOOT) when not RUNNING


class _FakeEsp32(object):
    class Partition(object):
        RUNNING = 1
        BOOT = 0
        def __new__(cls, which):
            if which == cls.BOOT and _boot_part[0] is not None:
                return _boot_part[0]
            return _slots[0]
        @staticmethod
        def mark_app_valid_cancel_rollback():
            _ota_log.append(("valid",))


_ota_log = []
_slots = [_FakePart("ota_0", 65536), _FakePart("ota_1", 65536)]
_ota_dir = tempfile.mkdtemp()
_ota_saved = (gl.esp32, gl.HTTP_GET, gl.OTA_URL, gl.OTA_KEY, gl.FW_BUILD,
              gl.FW_COMMIT, gl.OTA_PENDING, gl.OTA_BAD, gl.machine.reset,
              gl.DIAG, gl.OTA_TRIES, gl.OTA_PAGES)
_ota_saved2 = (getattr(gl, "OTA_MARK", None), getattr(gl, "OTA_SHADOW", None))
gl.esp32 = _FakeEsp32
gl.OTA_PENDING = os.path.join(_ota_dir, "ota.pending")
gl.OTA_BAD = os.path.join(_ota_dir, "ota.bad")
gl.OTA_TRIES = os.path.join(_ota_dir, "ota.tries")
gl.OTA_MARK = os.path.join(_ota_dir, "ota.mark")
gl.OTA_SHADOW = (os.path.join(_ota_dir, "gps_logger.mpy"),
                 os.path.join(_ota_dir, "gps_logger.py"))
gl.OTA_PAGES = (os.path.join(_ota_dir, "web.html"),
                os.path.join(_ota_dir, "web.html.gz"))
gl.machine.reset = lambda: _ota_log.append(("reset",))
gl.DIAG = 60000
_img = bytes((i * 7) & 255 for i in range(10000))      # 2 blocks + a tail
_page = b"<html>new page</html>"
_files = {}


def _ota_get(url, sink, limit):
    body = _files.get(url)
    if body is None:
        return 404
    if len(body) > limit:
        raise ValueError("too big")
    for i in range(0, len(body), 1000):
        sink(memoryview(body)[i:i + 1000])
    return 200


def _manifest(build, img=_img, key=b"secret", **kw):
    page_path = os.path.join(_ota_dir, "web.html.gz")
    m = {"build": build, "commit": "abc%04d" % build, "size": len(img),
         "sha256": _hl.sha256(img).hexdigest(), "url": "micropython.bin",
         "files": [{"path": page_path, "size": len(_page),
                    "sha256": _hl.sha256(_page).hexdigest(),
                    "url": "web.html.gz"}]}
    m.update(kw)
    m["hmac"] = gl._hmac_hex(key, gl._ota_signed(m))
    return m


def _serve(m, img=_img):
    _files.clear()
    _files["http://ota.lan/gps/manifest.json"] = json.dumps(m).encode()
    _files["http://ota.lan/gps/micropython.bin"] = img
    _files["http://ota.lan/gps/web.html.gz"] = _page


def _ota_reset():
    del _ota_log[:]
    _boot_part[0] = None
    for sl in _slots:
        sl.mem[:] = b"\0" * len(sl.mem)
        sl.boot = False
    for f in (gl.OTA_PENDING, gl.OTA_BAD, gl.OTA_TRIES,
              getattr(gl, "OTA_MARK", None) or os.path.join(_ota_dir, "-"),
              os.path.join(_ota_dir, "web.html.gz"),
              os.path.join(_ota_dir, "web.html.gz.new")):
        try:
            os.remove(f)
        except OSError:
            pass
    gl.ota.update(state="idle", pend=None, pend_at=None, wifi_seen=False,
                  at=None, paths=[], failed=None, last_err=None, said=None,
                  held_at=None, held_ok=None, stuck=False)
    diag_clear()


try:
    gl.HTTP_GET = _ota_get
    gl.OTA_URL, gl.OTA_KEY = "http://ota.lan/gps/manifest.json", b"secret"
    gl.FW_BUILD, gl.FW_COMMIT = 100, "abc0100"
    # a newer signed build: written to the other slot, read back, the page
    # replaced, set to boot, pending noted, the board reset
    _ota_reset()
    _serve(_manifest(200))
    gl._ota_check()
    slot = _slots[1].mem
    page_p = os.path.join(_ota_dir, "web.html.gz")
    with open(page_p + ".new", "rb") as f:
        page_ok = f.read() == _page and not os.path.exists(page_p)
    with open(gl.OTA_PENDING) as f:
        pend = f.read()
    check("OTA-INSTALL: a newer signed build goes to the other slot (0xFF "
          "padded), the page is staged (.new), the slot boots, pending noted",
          bytes(slot[:len(_img)]) == _img
          and bytes(slot[len(_img):12288]) == b"\xff" * (12288 - len(_img))
          and bytes(_slots[0].mem[:10]) == b"\0" * 10 and page_ok
          and _ota_log == [("boot", "ota_1"), ("reset",)]
          and pend == "200 abc0200\n" + page_p
          and any("ota abc0200 installed (was abc0100)" in l
                  for l in diag_lines()),
          repr((_ota_log, pend, diag_lines()[-2:])))
    # refused: an older/equal build, a bad HMAC, a sha256 that does not
    # match, too big for the slot, a failed build, no OTA slot
    states = []
    for case in ("old", "hmac", "sha", "big", "bad", "noslot", "404"):
        _ota_reset()
        if case == "old":
            _serve(_manifest(100))
        elif case == "hmac":
            _serve(_manifest(200, key=b"wrong"))
        elif case == "sha":
            _serve(_manifest(200), img=_img[:-1] + b"x")
        elif case == "big":
            big = b"z" * 70000
            _serve(_manifest(200, img=big), img=big)
        elif case == "bad":
            with open(gl.OTA_BAD, "w") as f:
                f.write("200\n")
            _serve(_manifest(200))
        elif case == "noslot":
            gl.esp32 = None
            _serve(_manifest(200))
        else:
            _files.clear()
        try:
            gl._ota_check()
        except Exception as e:
            states.append(("raised", repr(e)))
        else:
            states.append(gl.ota["state"])
        gl.esp32 = _FakeEsp32
        if _ota_log:
            states.append(list(_ota_log))
    check("OTA-REFUSE: older build, bad HMAC, sha256 mismatch, too big, a "
          "failed build, no slot API, no manifest: nothing boots",
          states == ["current", "bad hmac", "bad image", "too big", "current",
                     "no ota slot", "manifest 404"], repr(states))

    # pending: valid after OTA_VALID_AFTER with WiFi and the manifest read
    # (the staged page goes up); no WiFi in OTA_HEALTH: a reset (no
    # set_boot: the bootloader aborts the pending image), a try (OTA-NOWIFI);
    # the old build at boot is a rollback, blamed unless the board was
    # powered off
    page_p = os.path.join(_ota_dir, "web.html.gz")
    res = []
    for wifi_ok in (True, False):
        _ota_reset()
        _serve(_manifest(200))
        gl.FW_BUILD, gl.FW_COMMIT = 200, "abc0200"
        with open(page_p, "wb") as f:
            f.write(b"old page")
        with open(page_p + ".new", "wb") as f:
            f.write(_page)
        with open(gl.OTA_PENDING, "w") as f:
            f.write("200 abc0200\n" + page_p)
        gl.ota_boot()
        gl.wifi_reset()
        gl.wifi["state"] = "connected" if wifi_ok else "not found"
        gl.ota_tick(_clock[0])
        early = list(_ota_log)
        _clock[0] += gl.OTA_VALID_AFTER + 1000
        gl.ota_tick(_clock[0])
        mid = (list(_ota_log), gl.ota["state"])
        _clock[0] += gl.OTA_HEALTH
        gl.ota_tick(_clock[0])
        with open(page_p, "rb") as f:
            page = f.read()
        res.append((early, mid, list(_ota_log), gl.ota["state"],
                    os.path.exists(gl.OTA_PENDING), gl._ota_bad(), page,
                    os.path.exists(page_p + ".new")))
    rb = []
    for cause in (2, 1):                 # a hard reset, then a power-on
        _ota_reset()
        gl.FW_BUILD, gl.FW_COMMIT = 100, "abc0100"
        with open(page_p + ".new", "wb") as f:
            f.write(_page)
        with open(gl.OTA_PENDING, "w") as f:
            f.write("200 abc0200\n" + page_p)
        if cause == 2:                   # the new build crashed (run())
            with open(gl.OTA_MARK, "w") as f:
                f.write("crash")
        _reset_cause[0] = cause
        gl.ota_boot()
        rb.append((gl.ota["pend"], gl.ota["state"],
                   os.path.exists(gl.OTA_PENDING), gl._ota_bad(),
                   os.path.exists(page_p + ".new"), list(_ota_log),
                   [l for l in diag_lines() if l.startswith("ota ")]))
    _reset_cause[0] = 1
    _ota_reset()
    gl.ota_boot()                        # no pending note: confirm the image
    plain = list(_ota_log)
    check("OTA-HEALTH: valid after OTA_VALID_AFTER with WiFi, the staged "
          "page goes up; no WiFi in OTA_HEALTH resets without set_boot "
          "(a try, staged page dropped)",
          res[0][:2] == ([], ([("valid",)], "ok"))
          and res[0][3:] == ("ok", False, [], _page, False)
          and res[1][1] == ([], "pending")
          and res[1][2] == [("reset",)]
          and res[1][4:] == (False, [], b"old page", False),
          repr(res))
    check("OTA-ROLLBACK: the old build at boot after a crash is a "
          "bootloader rollback (bad); after a power-off tried again; the "
          "running image is confirmed; a plain boot confirms it too",
          rb[0][:6] == (None, "rolled back", False, [200], False, [("valid",)])
          and "rolled back by the bootloader" in rb[0][6][-1]
          and rb[1][:6] == (None, "rolled back", False, [], False, [("valid",)])
          and "after a power-off" in rb[1][6][-1]
          and plain == [("valid",)],
          repr((rb, plain)))
    # OTA-PROMOTE: a staged page that cannot be renamed over the old one
    # (a file system that does not rename over an existing file) goes up
    # after the old one is removed; one that cannot go up at all is in the
    # diag log (it was silent, the new firmware served the old page)
    prom = []
    real_rename = os.rename
    for mode in ("exists", "broken"):
        _ota_reset()
        _serve(_manifest(200))
        diag_clear()
        gl.FW_BUILD, gl.FW_COMMIT = 200, "abc0200"
        with open(page_p, "wb") as f:
            f.write(b"old page")
        with open(page_p + ".new", "wb") as f:
            f.write(_page)
        with open(gl.OTA_PENDING, "w") as f:
            f.write("200 abc0200\n" + page_p)

        def _rename(a, b, mode=mode):
            if a == page_p + ".new" and (mode == "broken"
                                         or os.path.exists(b)):
                raise OSError(17)
            return real_rename(a, b)

        gl.os.rename = _rename
        try:
            gl.ota_boot()
            gl.wifi_reset()
            gl.wifi["state"] = "connected"
            gl.ota_tick(_clock[0])
            _clock[0] += gl.OTA_VALID_AFTER + 1000
            gl.ota_tick(_clock[0])
        finally:
            gl.os.rename = real_rename
        page = None
        if os.path.exists(page_p):
            with open(page_p, "rb") as f:
                page = f.read()
        prom.append((gl.ota["state"], page,
                     [l for l in diag_lines() if "page" in l]))
    check("OTA-PROMOTE: a page not renamed over the old one goes up after "
          "moving it aside; one that cannot go up is a diag line and the "
          "old page stays",
          prom[0][:2] == ("ok", _page) and prom[0][2] == []
          and prom[1][:2] == ("ok", b"old page") and prom[1][2]
          and "not put up" in prom[1][2][0] and page_p in prom[1][2][0],
          repr(prom))
    # OTA-PROMOTE-PLAIN: the board keeps only the gzip page now: once the
    # valid firmware has put a staged web.html.gz up, the old plain
    # web.html (~70 KB of flash) goes. Not before (a rollback keeps the old
    # firmware's files), not when the manifest names web.html itself, not
    # when the gzip could not go up
    plain_p = os.path.join(_ota_dir, "web.html")
    pp = []
    for mode in ("gz", "both", "broken", "rollback"):
        _ota_reset()
        _serve(_manifest(200))
        gl.FW_BUILD, gl.FW_COMMIT = (100, "abc0100") if mode == "rollback" \
            else (200, "abc0200")
        for p, b in ((page_p, b"old gz"), (page_p + ".new", _page),
                     (plain_p, b"old plain")):
            with open(p, "wb") as f:
                f.write(b)
        staged = [page_p]
        if mode == "both":
            with open(plain_p + ".new", "wb") as f:
                f.write(b"new plain")
            staged.append(plain_p)
        with open(gl.OTA_PENDING, "w") as f:
            f.write("200 abc0200\n" + "\n".join(staged))

        def _rename(a, b, mode=mode):
            if mode == "broken" and a == page_p + ".new":
                raise OSError(5)
            return real_rename(a, b)

        gl.os.rename = _rename
        try:
            gl.ota_boot()
            if mode != "rollback":
                gl.wifi_reset()
                gl.wifi["state"] = "connected"
                gl.ota_tick(_clock[0])
                _clock[0] += gl.OTA_VALID_AFTER + 1000
                gl.ota_tick(_clock[0])
        finally:
            gl.os.rename = real_rename
        pp.append((gl.ota["state"], dev_file(_ota_dir, "web.html.gz"),
                   dev_file(_ota_dir, "web.html")))
        for p in (plain_p, plain_p + ".new"):
            if os.path.exists(p):
                os.remove(p)
    check("OTA-PROMOTE-PLAIN: a promoted web.html.gz removes the old plain "
          "web.html; kept when the manifest names it, when the gzip did not "
          "go up, and after a rollback",
          pp == [("ok", _page, None), ("ok", _page, b"new plain"),
                 ("ok", b"old gz", b"old plain"),
                 ("rolled back", b"old gz", b"old plain")], repr(pp))
    # set_boot failing: no pending note blames a build that never ran, the
    # staged page is dropped
    _ota_reset()
    gl.FW_BUILD, gl.FW_COMMIT = 100, "abc0100"
    _serve(_manifest(200))
    def _sb_fail():
        raise OSError(-1)
    _slots[1].set_boot = _sb_fail
    try:
        try:
            gl._ota_check()
            sbf = "no error"
        except OSError:
            sbf = "raised"
    finally:
        del _slots[1].set_boot
    check("OTA-SETBOOT: a failed set_boot leaves no pending note, no staged "
          "page and no reset",
          sbf == "raised" and not os.path.exists(gl.OTA_PENDING)
          and not os.path.exists(page_p + ".new") and _ota_log == [],
          repr((sbf, _ota_log)))
    # OTA-MANIFEST: the HMAC covers the fields joined by "|", unescaped: a
    # manifest re-split at a "|" kept the HMAC (a page path swallowing the
    # next file; a commit swallowing the firmware fields, the page then
    # installed as the firmware). The format stays (boards in the field read
    # the same manifests); its fields are checked instead: no "|", ints,
    # 64 hex sha256, only the page paths, each once
    def _resign(x):
        try:
            x["hmac"] = gl._hmac_hex(b"secret", gl._ota_signed(x))
        except (TypeError, ValueError, KeyError):
            pass
        return x
    page_h = os.path.join(_ota_dir, "web.html")
    mm = _manifest(200)
    mm["files"].append({"path": page_h, "size": len(_page),
                        "sha256": _hl.sha256(_page).hexdigest(),
                        "url": "web.html.gz"})
    _resign(mm)
    f1, f2 = mm["files"]
    va = dict(mm, files=[{"path": "|".join([f1["path"], str(f1["size"]),
                                            f1["sha256"], f1["url"],
                                            f2["path"]]),
                          "size": f2["size"], "sha256": f2["sha256"],
                          "url": f2["url"]}])
    vb = {"build": 200, "commit": "|".join([
              mm["commit"], str(mm["size"]), mm["sha256"], mm["url"],
              f1["path"], str(f1["size"]), f1["sha256"], f1["url"],
              f2["path"]]),
          "size": f2["size"], "sha256": f2["sha256"], "url": f2["url"],
          "files": [], "hmac": mm["hmac"]}
    collide = [gl._hmac_hex(b"secret", gl._ota_signed(x)) == mm["hmac"]
               for x in (va, vb)]
    odd = [va, vb]
    for k, v in (("build", "200"), ("size", 10000.0), ("sha256", "ab" * 31),
                 ("sha256", "AB" * 32), ("url", "a|b.bin"), ("commit", 7)):
        odd.append(_resign(dict(mm, **{k: v})))
    for fs in ([dict(f1, path="/main.py")], [f1, f1], [dict(f1, size="21")],
               [dict(f1, sha256="0" * 63 + "|")], [dict(f1, url="a|b")],
               ["/web.html"], {"path": f1["path"]}):
        odd.append(_resign(dict(mm, files=fs)))
    man = []
    for x in odd:
        _ota_reset()
        gl.FW_BUILD, gl.FW_COMMIT = 100, "abc0100"
        _serve(x)
        w0 = _slots[1].writes
        try:
            gl._ota_check()
            man.append((gl.ota["state"], list(_ota_log),
                        _slots[1].writes - w0))
        except Exception as e:
            man.append(("raised", repr(e)[:40]))
    _ota_reset()
    _serve(mm)
    gl._ota_check()
    good = list(_ota_log)
    check("OTA-MANIFEST: a re-split manifest (same HMAC), a field with '|', "
          "a non-int, a bad sha256, an unknown or repeated page is refused "
          "before any download; the published shape installs",
          collide == [True, True]
          and man == [("bad manifest", [], 0)] * len(odd)
          and good == [("boot", "ota_1"), ("reset",)],
          repr((collide, man, good)))

    # OTA-PAGEFAIL: a page that fails its sha256 (or cannot be written)
    # raised past ota["failed"]: the 2 MB firmware came again every check;
    # it waits OTA_RETRY like a bad firmware download
    _ota_reset()
    gl.FW_BUILD, gl.FW_COMMIT = 100, "abc0100"
    _serve(_manifest(200))
    _files["http://ota.lan/gps/web.html.gz"] = b"<html>other page</html>"
    fw = []
    for k in range(4):
        _clock[0] += gl.OTA_CHECK
        w0 = _slots[1].writes
        try:
            gl._ota_check()
        except Exception:
            pass
        fw.append(_slots[1].writes > w0)
    _clock[0] += gl.OTA_RETRY
    _files["http://ota.lan/gps/web.html.gz"] = _page
    gl._ota_check()
    check("OTA-PAGEFAIL: a page failing its sha256 is one firmware download "
          "per OTA_RETRY, not one per check; nothing boots meanwhile",
          fw == [True, False, False, False]
          and _ota_log == [("boot", "ota_1"), ("reset",)],
          repr((fw, _ota_log, gl.ota.get("failed"))))

    # OTA-POWERON: MicroPython 1.29 reports a brownout as PWRON_RESET, so a
    # build that browned out on a weak supply was rolled back, never blamed
    # and installed again 2 min after every boot. A power-on rollback is a
    # try (OTA_TRIES): the next install waits OTA_RETRY, the
    # OTA_TRIES_MAX-th rollback marks the build bad
    _ota_reset()
    gl.FW_BUILD, gl.FW_COMMIT = 100, "abc0100"
    _serve(_manifest(200))
    pw = []
    for boot in range(5):
        gl.ota.update(state="idle", pend=None, failed=None, at=None)
        _reset_cause[0] = 1              # PWRON_RESET (a brownout too)
        gl.ota_boot()
        del _ota_log[:]
        _clock[0] += gl.OTA_FIRST
        gl._ota_check()
        first = (gl.ota["state"], ("boot", "ota_1") in _ota_log)
        if not first[1]:
            _clock[0] += gl.OTA_RETRY
            gl._ota_check()
        pw.append(first + (("boot", "ota_1") in _ota_log,))
    check("OTA-POWERON: a build rolled back at power-on (a brownout) waits "
          "OTA_RETRY, and is bad after OTA_TRIES_MAX such rollbacks",
          [p[1:] for p in pw] == [(True, True), (False, True), (False, False),
                                  (False, False), (False, False)]
          and pw[1][0] == "rolled back" and pw[2][0] == "current"
          and gl._ota_bad() == [200],
          repr((pw, gl._ota_bad())))

    # OTA-NOWIFI: no WiFi in OTA_HEALTH rolled back and blamed the build
    # for good: a router outage excluded a good build forever. It is a try
    # too: the old firmware waits OTA_RETRY, the second one is bad
    _ota_reset()
    nw = []
    for k in range(2):
        gl.FW_BUILD, gl.FW_COMMIT = 200, "abc0200"
        with open(gl.OTA_PENDING, "w") as f:
            f.write("200 abc0200\n")
        gl.ota.update(state="idle", pend=None, failed=None, at=None)
        del _ota_log[:]
        gl.ota_boot()
        gl.wifi_reset()
        gl.wifi["state"] = "not found"
        gl.ota_tick(_clock[0])
        _clock[0] += gl.OTA_HEALTH + 1000
        gl.ota_tick(_clock[0])
        r = (list(_ota_log), gl._ota_bad())
        # the bootloader boots the old image: no pending note left
        gl.FW_BUILD, gl.FW_COMMIT = 100, "abc0100"
        gl.ota.update(state="idle", pend=None, failed=None, at=None)
        _reset_cause[0] = 2
        gl.ota_boot()
        _serve(_manifest(200))
        del _ota_log[:]
        _clock[0] += gl.OTA_FIRST
        gl._ota_check()
        nw.append(r + (gl.ota["state"], list(_ota_log)))
    gl.wifi_reset()
    check("OTA-NOWIFI: no WiFi in OTA_HEALTH resets; the build waits "
          "OTA_RETRY and is bad only after OTA_TRIES_MAX such rollbacks",
          nw == [([("reset",)], [], "rolled back", []),
                 ([("reset",)], [200], "current", [])],
          repr(nw))

    # OTA-PROBE: a pending firmware is valid only once it read the manifest
    # too (HTTP 200, parsed): a build with a broken update path rolls back
    # instead of leaving the board to USB updates. Read every OTA_PROBE,
    # OTA_HEALTH as before; no OTA_URL: valid with WiFi as before
    pr = []
    gets = []
    def _counting_get(url, sink, limit):
        gets.append(url)
        return _ota_get(url, sink, limit)
    gl.HTTP_GET = _counting_get
    try:
        for mode in ("served", "404", "late", "no url"):
            _ota_reset()
            del gets[:]
            gl.FW_BUILD, gl.FW_COMMIT = 200, "abc0200"
            with open(gl.OTA_PENDING, "w") as f:
                f.write("200 abc0200\n")
            if mode == "served":
                _serve(_manifest(200))
            else:
                _files.clear()
            url = gl.OTA_URL
            if mode == "no url":
                gl.OTA_URL = None
            try:
                gl.ota_boot()
                gl.wifi_reset()
                gl.wifi["state"] = "connected"
                gl.ota_tick(_clock[0])
                _clock[0] += gl.OTA_VALID_AFTER + 1000
                gl.ota_tick(_clock[0])
                mid = (list(_ota_log), gl.ota["state"])
                if mode == "late":
                    _serve(_manifest(200))
                for i in range(gl.OTA_HEALTH // 5000):
                    if gl.ota["pend"] is None:
                        break            # (valid or reset: the probe's end)
                    _clock[0] += 5000
                    gl.ota_tick(_clock[0])
            finally:
                gl.OTA_URL = url
            pr.append((mode, mid, list(_ota_log), gl._ota_bad(),
                       os.path.exists(gl.OTA_PENDING), len(gets)))
    finally:
        gl.HTTP_GET = _ota_get
        gl.wifi_reset()
    check("OTA-PROBE: valid after OTA_VALID_AFTER with WiFi and a manifest "
          "read; none read in OTA_HEALTH resets (a try, not bad); read "
          "every OTA_PROBE; no OTA_URL as before",
          [p[:5] for p in pr] == [
              ("served", ([("valid",)], "ok"), [("valid",)], [], False),
              ("404", ([], "pending"), [("reset",)], [], False),
              ("late", ([], "pending"), [("valid",)], [], False),
              ("no url", ([("valid",)], "ok"), [("valid",)], [], False)]
          and pr[0][5] == 1 and pr[3][5] == 0
          and 2 <= pr[1][5] <= gl.OTA_HEALTH // gl.OTA_PROBE + 2,
          repr(pr))
    # OTA-AWAKE: until the new firmware has read the manifest the radio
    # does not sleep (a failed first probe, then the idle sleep: no retry,
    # a rollback at OTA_HEALTH, and two such marked a good build bad), and
    # no low-memory or TLS reboot aborts the pending image (the bootloader
    # rolls it back, and older firmware blames the build)
    _ota_reset()
    gl.FW_BUILD, gl.FW_COMMIT = 200, "abc0200"
    with open(gl.OTA_PENDING, "w") as f:
        f.write("200 abc0200\n")
    _files.clear()
    gl.ota_boot()
    gl.wifi_reset()
    w_ = gl.network.WLAN(gl.network.STA_IF)
    gl.log_reset()
    saved_ld = gl.LOG_DIR
    gl.LOG_DIR = _ota_dir
    _resets_before = len(_ota_log)
    try:
        gl._wifi_round_failed(w_)
        aw_round = gl.wifi["state"]
        gl.up["enomem"] = gl.TLS_REBOOT_N
        gl.wifi["home"] = True
        gl.wifi["state"] = "connected"
        aw_tls = gl._tls_reboot()
        aw_low = gl._low_reboot("mem %d B", 4000, gl.LOWIDF_WAIT + 1000)
        aw_reset = list(_ota_log[_resets_before:])
    finally:
        gl.LOG_DIR = saved_ld
        gl.up["enomem"] = 0
        gl.wifi["home"] = False
        gl.wifi_reset()
    check("OTA-AWAKE: while a new firmware waits to read the manifest a "
          "failed round searches on (no sleep), and no TLS or low-memory "
          "reboot aborts it",
          aw_round == "not found" and gl.ota["pend"] == 200
          and aw_tls is False and aw_low is False and aw_reset == [],
          repr((aw_round, aw_tls, aw_low, aw_reset)))
    _ota_reset()
    # a reset in between keeps the state: _ota_env leaves "pending" alone
    gl.ota["state"] = "pending"
    gl._ota_env({"OTA_URL": "http://ota.lan/m.json", "OTA_KEY": "k"})
    st_keep = gl.ota["state"]
    gl.ota["state"] = "off"
    gl._ota_env({"OTA_URL": "http://ota.lan/m.json", "OTA_KEY": "k"})
    check("OTA-STATE: .env keeps a pending / rolled back state for the page",
          st_keep == "pending" and gl.ota["state"] == "idle", st_keep)

    # _raw_get: the body ends at Content-Length (TLS without close_notify
    # raised at EOF); a short body or one over the limit raises
    class _GetSock(object):
        def __init__(self, resp, eof_err=None):
            self.resp, self.rd, self.eof_err = resp, 0, eof_err
        def settimeout(self, t):
            pass
        def connect(self, a):
            pass
        def write(self, b):
            return len(b)
        def readline(self, n=-1):
            i = self.resp.find(b"\n", self.rd)
            i = len(self.resp) if i < 0 else i + 1
            if n is not None and n >= 0:
                i = min(i, self.rd + n)
            line, self.rd = self.resp[self.rd:i], i
            return line
        def readinto(self, mv):
            if self.rd >= len(self.resp):
                if self.eof_err:
                    raise self.eof_err
                return 0
            k = min(len(mv), len(self.resp) - self.rd)
            mv[:k] = self.resp[self.rd:self.rd + k]
            self.rd += k
            return k
        def close(self):
            pass
    _gs = [None]
    _gsock = types.ModuleType("socket")
    _gsock.SOCK_STREAM = 1
    _gsock.getaddrinfo = lambda h, p, a=0, b=0: [(2, 1, 6, "", (h, p))]
    _gsock.socket = lambda *a: _gs[0]
    _gsaved = (gl.socket, gl.read_uart)
    gl.socket, gl.read_uart = _gsock, lambda: None
    got = []
    try:
        body = b"y" * 3000
        _gs[0] = _GetSock(b"HTTP/1.0 200 OK\r\nContent-Length: 3000\r\n\r\n"
                          + body, eof_err=OSError(-0x7280))
        out = bytearray()
        got.append((gl._raw_get("http://h/x", out.extend, 5000),
                    bytes(out) == body))
        for resp in (b"HTTP/1.0 200 OK\r\nContent-Length: 3000\r\n\r\n" + b"y" * 10,
                     b"HTTP/1.0 200 OK\r\nContent-Length: 9000\r\n\r\n" + b"y"):
            _gs[0] = _GetSock(resp)
            try:
                gl._raw_get("http://h/x", bytearray().extend, 5000)
                got.append("ok")
            except ValueError as e:
                got.append(str(e)[:12])
        _gs[0] = _GetSock(b"HTTP/1.0 200 OK\r\n\r\nabc")
        out = bytearray()
        got.append((gl._raw_get("http://h/x", out.extend, 5000), bytes(out)))
        # HOST-PORT: the GET's Host header too
        get_hosts = []
        for u_ in ("http://h:8080/m.json", "http://h/m.json"):
            _gs[0] = _GetSock(b"HTTP/1.0 404 Not Found\r\n\r\n")
            sent_ = []
            _gs[0].write = lambda b, s_=sent_: s_.append(bytes(b)) or len(b)
            gl._raw_get(u_, bytearray().extend, 5000)
            get_hosts.append(b"".join(sent_).split(b"\r\n")[1])
    finally:
        gl.socket, gl.read_uart = _gsaved
    check("OTA-CLEN: reads exactly Content-Length (no EOF error), short or "
          "too long raises, no length reads to EOF",
          got == [(200, True), "ota body 10 ", "ota body 900", (200, b"abc")],
          repr(got))
    check("HOST-PORT: OTA GET Host header has a non-default port",
          get_hosts == [b"Host: h:8080", b"Host: h"], repr(get_hosts))

    # when it checks: connected at home with all uploaded, OTA_FIRST after
    # boot, then OTA_CHECK; never with points waiting or away
    _ota_reset()
    gl.FW_BUILD = 100
    calls = []
    saved_check = gl._ota_check
    gl._ota_check = lambda: calls.append(_clock[0])
    try:
        gl.wifi_reset()
        gl.log_reset()
        gl.up_reset()
        gl.wifi.update(state="connected", home=True)
        gl.ota["boot_at"] = _clock[0]
        gl.ota_tick(_clock[0] + gl.OTA_FIRST - 1000)
        n_early = len(calls)
        gl.ota_tick(_clock[0] + gl.OTA_FIRST)
        gl.ota_tick(_clock[0] + gl.OTA_FIRST + 60000)
        n_first = len(calls)
        gl.wifi["home"] = False
        gl.ota_tick(_clock[0] + gl.OTA_FIRST + gl.OTA_CHECK)
        n_away = len(calls)
        gl.wifi["home"] = True
        gl.up_reset()
        gl.up["left"] = 30               # points waiting in a file
        gl.ota_tick(_clock[0] + gl.OTA_FIRST + gl.OTA_CHECK)
        n_busy = len(calls)
        gl.up["left"] = 0
        gl.log["batch"].append("x")      # RAM only: flushed before a reset
        gl.ota_tick(_clock[0] + gl.OTA_FIRST + gl.OTA_CHECK)
        n_day = len(calls)
        gl.log["batch"][:] = []
    finally:
        gl._ota_check = saved_check
        gl.wifi_reset()
        gl.log_reset()
    check("OTA-TICK: first check OTA_FIRST after boot, then OTA_CHECK; not "
          "away from home, not with points waiting in files (RAM is fine)",
          (n_early, n_first, n_away, n_busy, n_day) == (0, 1, 1, 1, 2),
          repr((n_early, n_first, n_away, n_busy, n_day)))

    # .env: both keys or none; JSON: fw and ota
    gl.OTA_URL = gl.OTA_KEY = None
    gl._ota_env({"OTA_URL": "http://ota.lan/m.json"})
    one = (gl.OTA_URL, gl.OTA_KEY)
    gl._ota_env({"OTA_URL": "http://ota.lan/m.json", "OTA_KEY": "k3y "})
    both = (gl.OTA_URL, gl.OTA_KEY)
    jd = json.loads(gl._web_json())
    check("OTA-ENV: OTA_URL and OTA_KEY both or no updates; fw/ota in JSON",
          one == (None, None) and both == ("http://ota.lan/m.json", b"k3y")
          and jd.get("fw") == gl.FW_COMMIT and "ota" in jd,
          repr((one, both, jd.get("fw"), jd.get("ota"))))

    # --- OTA review 5: external resets, Ctrl-C, a shadowing .mpy, the
    # health timeout, the probe's HMAC, diag injection, the HMAC compare,
    # an upload stuck in an error ------------------------------------------
    import inspect as _insp
    gl.OTA_URL, gl.OTA_KEY = "http://ota.lan/gps/manifest.json", b"secret"
    gl.HTTP_GET = _ota_get
    page_p = os.path.join(_ota_dir, "web.html.gz")

    def _pend(build=200):
        with open(gl.OTA_PENDING, "w") as f:
            f.write("%d abc%04d\n" % (build, build))

    def _set_mark(what):
        with open(gl.OTA_MARK, "w") as f:
            f.write(what)

    def _guarded(fn):
        """A test body; one that raises (the fix missing) is its repr."""
        try:
            return fn()
        except Exception as e:
            return ("raised", repr(e)[:80])

    # OTA-EXTRESET: any reset in the pending window but a power-on (an
    # `mpremote reset`, a USB-JTAG reset, the WDT after an mpremote session
    # stopped the logger) was "rolled back by the bootloader": the build bad
    # at once. A crash (run()'s crash branch marks it in OTA_MARK) or a WDT
    # while the logger ran is still bad; anything else is a try
    def _t_ext():
        out = []
        for cause, mark in ((2, None), (0, None), (2, "stop"), (3, "stop"),
                            (2, "crash"), (3, None), (1, "crash")):
            _ota_reset()
            gl.FW_BUILD, gl.FW_COMMIT = 100, "abc0100"
            _pend()
            if mark:
                _set_mark(mark)
            _reset_cause[0] = cause
            gl.ota_boot()
            out.append((gl._ota_bad(), gl._ota_tries(),
                        os.path.exists(gl.OTA_MARK),
                        os.path.exists(gl.OTA_PENDING)))
        _reset_cause[0] = 1
        _ota_reset()
        gl._ota_mark("stop")             # no pending note: nothing written
        no_note = os.path.exists(gl.OTA_MARK)
        _pend()
        gl._ota_mark("crash")
        with open(gl.OTA_MARK) as f:
            mk = f.read()
        gl.FW_BUILD, gl.FW_COMMIT = 200, "abc0200"
        gl.ota_boot()                    # the pending run starts: cleared
        src = _insp.getsource(gl.run)
        return (out, no_note, mk, os.path.exists(gl.OTA_MARK),
                '_ota_mark("crash")' in src, '_ota_mark("stop")' in src)
    ext = _guarded(_t_ext)
    try_ = ([], (200, 1), False, False)
    bad_ = ([200], (None, 0), False, False)
    check("OTA-EXTRESET: an external reset in the pending window (hard, "
          "USB-JTAG, WDT after a Ctrl-C) is a try; a crash or a WDT while "
          "the logger ran is bad; run() marks crash / stop",
          ext == ([try_] * 4 + [bad_] * 3, False, "crash", False, True, True),
          repr(ext))

    # OTA-CTRLC: Ctrl-C (a BaseException) after set_boot skipped the reset
    # (the old firmware then blamed the build, and the next reset booted it
    # with no pending note: valid without a health check); before set_boot
    # it left the note, and a build that never ran became bad
    def _t_ctrlc():
        out = []
        real_flush = gl.flush

        def _kb(*a, **k):
            raise KeyboardInterrupt
        for case in ("flush", "set_boot", "set_boot done"):
            _ota_reset()
            gl.FW_BUILD, gl.FW_COMMIT = 100, "abc0100"
            _serve(_manifest(200))
            if case == "flush":
                gl.flush = _kb
            elif case == "set_boot":
                _slots[1].set_boot = _kb
            else:
                def _sb_done():
                    _boot_part[0] = _slots[1]
                    raise KeyboardInterrupt
                _slots[1].set_boot = _sb_done
            try:
                try:
                    gl._ota_check()
                    r = "returned"
                except KeyboardInterrupt:
                    r = "raised"
            finally:
                gl.flush = real_flush
                if "set_boot" in _slots[1].__dict__:
                    del _slots[1].set_boot
            out.append((r, list(_ota_log), os.path.exists(gl.OTA_PENDING),
                        os.path.exists(page_p + ".new")))
        # interrupted before set_boot: a restarted run() blames nothing
        _ota_reset()
        _serve(_manifest(200))
        _slots[1].set_boot = _kb
        try:
            gl._ota_check()
        except KeyboardInterrupt:
            pass
        finally:
            del _slots[1].set_boot
        gl.ota_boot()
        out.append(gl._ota_bad())
        # set_boot done, no reset since (the boot slot is not the running
        # one): ota_boot resets into it, once; still not run after that: a
        # try, no reset loop
        _ota_reset()
        gl.FW_BUILD, gl.FW_COMMIT = 100, "abc0100"
        _pend()
        _boot_part[0] = _slots[1]
        _reset_cause[0] = 2
        gl.ota_boot()
        out.append((list(_ota_log), os.path.exists(gl.OTA_PENDING),
                    gl._ota_bad(), gl._ota_tries()))
        del _ota_log[:]
        gl.ota_boot()
        out.append((list(_ota_log), os.path.exists(gl.OTA_PENDING),
                    gl._ota_bad(), gl._ota_tries()))
        _reset_cause[0] = 1
        _boot_part[0] = None
        return out
    cc = _guarded(_t_ctrlc)
    check("OTA-CTRLC: Ctrl-C after set_boot still resets; before it the note "
          "and staged page go and nothing is blamed; a boot slot set but not "
          "run is booted once, then a try",
          cc == [("returned", [("boot", "ota_1"), ("reset",)], True, True),
                 ("raised", [], False, False),
                 ("returned", [("reset",)], True, True),
                 [],
                 ([("reset",)], True, [], (None, 0)),
                 ([("valid",)], False, [], (200, 1))],
          repr(cc))

    # OTA-SHADOW: a /gps_logger.mpy (FORCE_MPY=1) shadows the frozen logger
    # ('' comes before .frozen in sys.path): the diag said "ota <new> ok"
    # while the old code ran. The install moves it (and a .py) aside as
    # .ota-old, the valid new firmware drops it, a failed install puts it
    # back (a rollback: main.py, before the import)
    def _t_shadow():
        out = []
        mpy = gl.OTA_SHADOW[0]
        for case in ("ok", "set_boot fails", "not frozen"):
            _ota_reset()
            for p in (mpy, mpy + ".ota-old"):
                if os.path.exists(p):
                    os.remove(p)
            with open(mpy, "wb") as f:
                f.write(b"MPY emergency")
            gl.FW_BUILD, gl.FW_COMMIT = (0 if case == "not frozen"
                                         else 100), "abc0100"
            _serve(_manifest(200))
            if case == "set_boot fails":
                def _sbf():
                    raise OSError(-1)
                _slots[1].set_boot = _sbf
            try:
                gl._ota_check()
            except OSError:
                pass
            finally:
                if "set_boot" in _slots[1].__dict__:
                    del _slots[1].set_boot
            r = [os.path.exists(mpy), os.path.exists(mpy + ".ota-old")]
            if case == "ok":             # the new firmware becomes valid
                gl.FW_BUILD, gl.FW_COMMIT = 200, "abc0200"
                gl.ota_boot()
                gl.wifi_reset()
                gl.wifi["state"] = "connected"
                gl.ota_tick(_clock[0])
                _clock[0] += gl.OTA_VALID_AFTER + 1000
                gl.ota_tick(_clock[0])
                gl.wifi_reset()
                r += [gl.ota["state"], os.path.exists(mpy),
                      os.path.exists(mpy + ".ota-old")]
            out.append(r)
        return out
    sh = _guarded(_t_shadow)
    check("OTA-SHADOW: the install moves a shadowing gps_logger.mpy aside "
          "(.ota-old), the valid firmware drops it, a failed install puts "
          "it back; not on a plain (non-frozen) firmware",
          sh == [[False, True, "ok", False, False], [True, False],
                 [True, False]], repr(sh))
    # main.py puts it back before the import when the pending build is not
    # the one running (rolled back) or the note is gone (the health reset)
    def _t_shadow_main():
        mp = os.path.join(ROOT, "main.py")
        import gps_boot as gb
        gb_saved = (gb.OTA_SHADOW, gb.OTA_PENDING)
        gb.OTA_SHADOW, gb.OTA_PENDING = gl.OTA_SHADOW, gl.OTA_PENDING
        mpy = gl.OTA_SHADOW[0]
        fz = types.ModuleType("gps_frozen")
        prev = sys.modules.get("gps_frozen")
        sys.modules["gps_frozen"] = fz
        out = []
        try:
            for running, note, orig in ((200, 200, False), (100, 200, False),
                                        (100, None, False), (100, None, True)):
                _ota_reset()
                for p in (mpy, mpy + ".ota-old"):
                    if os.path.exists(p):
                        os.remove(p)
                with open(mpy + ".ota-old", "wb") as f:
                    f.write(b"MPY emergency")
                if orig:
                    with open(mpy, "wb") as f:
                        f.write(b"MPY deployed since")
                if note:
                    _pend(note)
                fz.BUILD = running
                gb.unshadow()
                with open(mpy, "rb") if os.path.exists(mpy) else \
                        io.BytesIO(b"") as f:
                    body = f.read()
                out.append((body, os.path.exists(mpy + ".ota-old")))
        finally:
            gb.OTA_SHADOW, gb.OTA_PENDING = gb_saved
            if prev is None:
                sys.modules.pop("gps_frozen", None)
            else:
                sys.modules["gps_frozen"] = prev
        src = open(mp).read()
        return out, src.index("gps_boot.unshadow()") < src.index(
            "import gps_logger")
    shm = _guarded(_t_shadow_main)
    check("OTA-SHADOW-MAIN: main.py puts the moved-aside .mpy back before "
          "the import unless the pending build is the one running",
          shm == ([(b"", True), (b"MPY emergency", False),
                   (b"MPY emergency", False),
                   (b"MPY deployed since", False)], True), repr(shm))

    # OTA-HEALTH-ORDER: (a) a power cut between _ota_try and removing the
    # note counted the one attempt twice (bad at once); (b) the RAM batch
    # was not flushed before the health-timeout reset
    def _t_order():
        _ota_reset()
        gl.FW_BUILD, gl.FW_COMMIT = 200, "abc0200"
        _pend()
        gl.ota_boot()
        gl.wifi_reset()
        gl.wifi["state"] = "not found"
        real_rm = gl._rm

        def _rm_cut(p):
            if p == gl.OTA_PENDING:
                raise SystemExit("power cut")
            return real_rm(p)
        gl._rm = _rm_cut
        _clock[0] += gl.OTA_HEALTH + 1000
        try:
            gl.ota_tick(_clock[0])
        except SystemExit:
            pass
        finally:
            gl._rm = real_rm
        gl.FW_BUILD, gl.FW_COMMIT = 100, "abc0100"
        _reset_cause[0] = 1
        gl.ota_boot()
        a = (gl._ota_tries(), gl._ota_bad())
        _ota_reset()
        gl.FW_BUILD, gl.FW_COMMIT = 200, "abc0200"
        _pend()
        gl.ota_boot()
        gl.wifi["state"] = "not found"
        flushed = []
        real_flush = gl.flush
        gl.flush = lambda *x, **k: flushed.append(len(_ota_log))
        try:
            _clock[0] += gl.OTA_HEALTH + 1000
            gl.ota_tick(_clock[0])
        finally:
            gl.flush = real_flush
            gl.wifi_reset()
        return a, flushed, list(_ota_log)
    od = _guarded(_t_order)
    check("OTA-HEALTH-ORDER: a power cut in the health timeout counts one "
          "try, not two; the RAM batch is flushed before its reset",
          od == (((200, 1), []), [0], [("reset",)]), repr(od))
    # OTA-HEALTH-RESET: an error between ending the pending state and the
    # reset (a MemoryError formatting the diag line, in _ota_try) left the
    # image running with no note and no health check; the reset is in a
    # finally now
    def _t_hreset():
        _ota_reset()
        gl.FW_BUILD, gl.FW_COMMIT = 200, "abc0200"
        _pend()
        gl.ota_boot()
        gl.wifi_reset()
        gl.wifi["state"] = "not found"
        real_try = gl._ota_try
        gl._ota_try = lambda b: (_ for _ in ()).throw(MemoryError())
        _clock[0] += gl.OTA_HEALTH + 1000
        try:
            gl.ota_tick(_clock[0])
            r = "returned"
        except MemoryError:
            r = "raised"
        finally:
            gl._ota_try = real_try
            gl.wifi_reset()
        return r, list(_ota_log)
    hr = _guarded(_t_hreset)
    check("OTA-HEALTH-RESET: the health timeout resets even when a step "
          "before the reset fails", hr[1] == [("reset",)], repr(hr))

    # OTA-PROBE-HMAC: the pending firmware's probe took any manifest with
    # the right fields: one signed with another key made it valid
    def _t_probe():
        _ota_reset()
        gl.FW_BUILD, gl.FW_COMMIT = 200, "abc0200"
        _pend()
        _serve(_manifest(200, key=b"wrong"))
        gl.ota_boot()
        gl.wifi_reset()
        gl.wifi["state"] = "connected"
        try:
            for i in range((gl.OTA_HEALTH + gl.OTA_VALID_AFTER) // 5000 + 2):
                if gl.ota["pend"] is None:
                    break
                gl.ota_tick(_clock[0])
                _clock[0] += 5000
        finally:
            gl.wifi_reset()
        return list(_ota_log), gl.ota.get("probe_err")
    pb = _guarded(_t_probe)
    check("OTA-PROBE-HMAC: a manifest whose HMAC does not check does not make "
          "a pending firmware valid", pb[0] == [("reset",)]
          and "hmac" in str(pb[1]), repr(pb))

    # OTA-DIAG: the commit (an unverified field) went to the diag log on an
    # HMAC mismatch, newlines and all (forged lines), every check; the
    # commit is now hex (+ "-dirty"), up to 20; _diag_write keeps a line
    # one line, capped
    def _t_diag():
        st = []
        for commit in ("x\nota abc0300 ok\nfake", "a" * 21, "abc 123",
                       "ABC1234", ""):
            _ota_reset()
            gl.FW_BUILD, gl.FW_COMMIT = 100, "abc0100"
            _serve(_manifest(200, commit=commit))
            gl._ota_check()
            st.append(gl.ota["state"])
        okd = [gl._ota_manifest_ok(_manifest(200, commit=c))
               for c in ("abc1234-dirty", "0123456789abcdef0123", "6b62646")]
        _ota_reset()
        gl.FW_BUILD, gl.FW_COMMIT = 100, "abc0100"
        _serve(_manifest(200, key=b"wrong", commit="deadbee"))
        gl._ota_check()
        _clock[0] += gl.OTA_CHECK
        gl._ota_check()
        hm = [ln for ln in diag_lines() if "hmac" in ln]
        # (a PVT status line runs to ~220 B after weeks up: the cap is
        # above it, or its last field, hacc, was cut off the page's donut)
        diag_clear()
        gl._diag_write("s" * 260)
        keep = diag_lines() == ["s" * 260]
        diag_clear()
        gl._diag_write("a\nb\rc" + "x" * 500)
        dl = diag_lines()
        return (st, okd, len(hm), any("deadbee" in ln for ln in hm),
                len(dl), max(len(ln) for ln in dl) <= gl.DIAG_CUT, keep)
    dg = _guarded(_t_diag)
    check("OTA-DIAG: a commit that is not one is refused (signed or not); "
          "an HMAC mismatch is one diag line without unverified fields; a "
          "diag line has no line breaks and is capped",
          dg == (["bad manifest"] * 5, [True, True, True], 1, False, 1, True,
                 True),
          repr(dg))

    # OTA-CTEQ: the HMAC compare is constant-time (XOR-accumulated), not !=
    def _t_cteq():
        eq = gl._ota_eq
        return ([eq("ab12", "ab12"), eq("ab12", "ab13"), eq("ab12", "ab1"),
                 eq("ab12", None), eq("ab12", 12)],
                '!= m.get("hmac")' in _insp.getsource(gl._ota_check))
    ct = _guarded(_t_cteq)
    check("OTA-CTEQ: the manifest HMAC is compared in constant time",
          ct == ([True, False, False, False, False], False), repr(ct))

    # OTA-STUCK: an upload error that never clears ("err payload" is only
    # cleared by a 2xx) held every check back: an upload bug blocked the
    # OTA that fixes it. After OTA_STUCK with no 2xx it checks anyway
    def _t_stuck():
        _ota_reset()
        gl.FW_BUILD = 100
        calls = []
        saved_check = gl._ota_check
        gl._ota_check = lambda: calls.append(1)
        h = 3600000
        try:
            gl.wifi_reset()
            gl.log_reset()
            gl.up_reset()
            gl.wifi.update(state="connected", home=True)
            gl.up["state"] = "err payload"
            gl.up["left"] = 5
            t0 = _clock[0]
            gl.ota["boot_at"] = t0
            n = []
            for t in (gl.OTA_FIRST, 23 * h):
                gl.ota_tick(t0 + t)
                n.append(len(calls))
            gl.up["ok_at"] = t0 + 23 * h     # a 2xx: it is moving
            gl.ota_tick(t0 + 25 * h)
            n.append(len(calls))
            gl.ota_tick(t0 + 50 * h)
            n.append(len(calls))
            gl.wifi["home"] = False          # away: never
            gl.ota_tick(t0 + 51 * h)
            n.append(len(calls))
        finally:
            gl._ota_check = saved_check
            gl.wifi_reset()
            gl.log_reset()
            gl.up_reset()
        return n, gl.OTA_STUCK, [ln for ln in diag_lines() if "stuck" in ln]
    sk = _guarded(_t_stuck)
    check("OTA-STUCK: an upload error with no 2xx for OTA_STUCK (24 h) no "
          "longer holds the update check back for good",
          sk[:2] == ([0, 0, 0, 1, 1], 86400000) and len(sk[2]) == 1,
          repr(sk))
    # BOOT-SAFE: after BOOT_FAIL_MAX the board sat in the REPL for good, no
    # watchdog, no logging. It retries after BOOT_RETRY_MS (one more start:
    # the count set back to BOOT_FAIL_MAX - 1); the REPL is free until then
    # (the timer can be stopped), and the boot-bypass pin skips it all
    def _t_safe():
        mp = os.path.join(ROOT, "main.py")
        ns = {"__name__": "boot_safe"}
        exec(compile(open(mp).read(), mp, "exec"), ns)
        out = []
        for how in ("start", "error"):
            del _timers[:]
            del _ota_log[:]
            RTC.mem = b"%d" % (ns["BOOT_FAIL_MAX"] - (how == "error"))
            if how == "start":
                ns["_boot_start"]()
            else:
                out.append(ns["_boot_handle_error"]())
            t = [x for x in _timers if x.callback]
            per = t[0].period if t else None
            if t and t[0].id != -1:
                per = ("hardware timer", t[0].id)
            mem0 = RTC.mem
            if t:
                t[0].callback(t[0])
            out.append((per, mem0, RTC.mem, list(_ota_log)))
        RTC.mem = b""
        return out, ns["BOOT_RETRY_MS"]
    sf = _guarded(_t_safe)
    check("BOOT-SAFE: at BOOT_FAIL_MAX a timer retries the logger after "
          "BOOT_RETRY_MS (count back to BOOT_FAIL_MAX - 1, a reset)",
          sf == ([(1800000, b"5", b"4", [("reset",)]), False,
                  (1800000, b"5", b"4", [("reset",)])], 1800000), repr(sf))
    # MAIN-SLIM: main.py is compiled on the board at every boot (its code
    # in the heap for the whole run, ~5 KB more while it compiles): the OTA
    # and retry helpers live in the frozen gps_boot, the retry timer is a
    # virtual one (hardware Timer(0) is the logger's Ctrl-C keep-alive)
    with open(os.path.join(ROOT, "main.py")) as f:
        msrc = f.read()
    check("MAIN-SLIM: main.py leaves the OTA and retry code to gps_boot",
          "import gps_boot" in msrc and "def _ota_unshadow" not in msrc
          and "Timer(" not in msrc and len(msrc) < 4000, len(msrc))
    # MAIN-CRASH: a pending image that fails in main.py (its import, or
    # run() before its own crash branch) reset without OTA_MARK: the old
    # firmware took it for an external reset, a try, and installed the bad
    # build again OTA_TRIES_MAX times. main.py now marks it "crash"
    def _t_main_crash():
        import gps_boot as gb
        mp = os.path.join(ROOT, "main.py")
        ns = {"__name__": "boot_crash"}
        exec(compile(open(mp).read(), mp, "exec"), ns)
        saved = (gb.OTA_PENDING, gb.OTA_MARK, ns["time"].sleep)
        gb.OTA_PENDING, gb.OTA_MARK = gl.OTA_PENDING, gl.OTA_MARK
        ns["time"].sleep = lambda s: None
        out = []
        try:
            for pend in (True, False):
                _ota_reset()
                if pend:
                    _pend(200)
                RTC.mem = b"0"
                ns["_boot_handle_error"]()
                out.append(gl._ota_mark_get())
        finally:
            gb.OTA_PENDING, gb.OTA_MARK, ns["time"].sleep = saved
            RTC.mem = b""
        return out
    mc = _guarded(_t_main_crash)
    check("MAIN-CRASH: a logger failing in main.py while an OTA is pending "
          "marks the build's run as a crash", mc == ["crash", None], repr(mc))
    # OTA-DOC: the C6's boot-bypass pin is GPIO10, not GPIO25
    with open(os.path.join(ROOT, "tests", "device_bench.py")) as f:
        bench = f.read()
    with open(os.path.join(ROOT, "gps_logger.py")) as f:
        gsrc = f.read()
    check("OTA-DOC: no comment says the boot bypass is GPIO25 only",
          "GPIO25 boot-bypass" not in gsrc and "GPIO25" not in bench,
          "")
finally:
    (gl.esp32, gl.HTTP_GET, gl.OTA_URL, gl.OTA_KEY, gl.FW_BUILD,
     gl.FW_COMMIT, gl.OTA_PENDING, gl.OTA_BAD, gl.machine.reset,
     gl.DIAG, gl.OTA_TRIES, gl.OTA_PAGES) = _ota_saved
    for _n, _v in zip(("OTA_MARK", "OTA_SHADOW"), _ota_saved2):
        if _v is None:
            if hasattr(gl, _n):
                delattr(gl, _n)
        else:
            setattr(gl, _n, _v)
    _reset_cause[0] = 1
    diag_clear()
    shutil.rmtree(_ota_dir, ignore_errors=True)

# --- second review (R2-n) ---------------------------------------------------
_r2_dir = tempfile.mkdtemp()
gl.wifi_reset()
gl.LOG_DIR = _r2_dir
gl.log_reset()
gl.up_reset()

# R2-3: the WFAIL_SWITCH file change keeps the queued segment breaks
gl.start_file("20261101_120000")
gl.log["batch"].extend(trk_line(47.1, 18.4, "2026-11-01T12:00:%02dZ" % i)
                       for i in range(2))
gl.flush()
f_old = gl.log["file"]
_o3 = builtins.open
def _bad3(name, mode="r", *a, **k):
    if name == f_old and mode != "rb":
        raise OSError(5)
    return _o3(name, mode, *a, **k)
builtins.open = _bad3
try:
    for r in range(gl.WFAIL_SWITCH):
        if r == 1:
            gl._seg_break()              # a fix loss while writes fail
        gl._record((r, trk_line(47.2, 18.4, "2026-11-01T12:05:%02dZ" % r)),
                   (47.2, 18.4))
        gl.flush()
finally:
    builtins.open = _o3
gl.flush()
with open(gl.log["file"]) as f:
    seg3 = f.read()
check("R2-3: a file change after failed writes keeps a fix loss as a "
      "segment break in the new file",
      gl.log["file"] != f_old and "-\n" in seg3
      and seg3.index("120500Z") < seg3.index("-\n") < seg3.index("120501Z"),
      repr(seg3))
gl.log_reset()

# R2-7: v1.29's [] is a failed scan once; the next empty ones in a row are a
# place without APs (no every-network round each time; a stay counts them)
class _EmptyW(object):
    def scan(self):
        return []
    def active(self, x=None):
        return True
gl.wifi_reset()
gl.wifi["nets"] = [{"ssid": "home", "pw": "x", "home": True,
                    "hidden": False, "bssid": None, "ip": None}]
stays = []
_ss = gl._stay_scan
gl._stay_scan = lambda aps: stays.append(len(aps))
try:
    e7 = [gl._scan_known(_EmptyW()), gl._scan_known(_EmptyW())]
    gl.wifi["empty"] = 0
    gl._pb_pulse_radio(_EmptyW(), True)
    first_pulse = list(stays)
    gl._pb_pulse_radio(_EmptyW(), True)
finally:
    gl._stay_scan = _ss
check("R2-7: one empty scan is a failed one, two in a row a place without "
      "APs (same rule for the keep-alive scans)",
      e7 == [None, []] and first_pulse == [0] and stays == [0, 0],
      repr((e7, first_pulse, stays)))
gl.wifi_reset()

# R2-8: KEEPALIVE_MODE=gpio on the default pin the GPS moved onto falls back
# to scan
_k8 = (gl.PB_MODE, gl.PB_PIN, gl.GPS_TX, gl.GPS_RX)
try:
    gl.PB_PIN, gl.GPS_TX, gl.GPS_RX = 2, 3, 2
    gl._keepalive_env({"KEEPALIVE_MODE": "gpio"})
    m8a = gl.PB_MODE
    gl.GPS_TX, gl.GPS_RX = 18, 19
    gl._keepalive_env({"KEEPALIVE_MODE": "gpio"})
    m8b = gl.PB_MODE
finally:
    gl.PB_MODE, gl.PB_PIN, gl.GPS_TX, gl.GPS_RX = _k8
check("R2-8: gpio keep-alive on a GPS pin falls back to scan",
      (m8a, m8b) == ("scan", "gpio"), repr((m8a, m8b)))

# R2-PVT: GPS_PVT takes the usual false words
_pv = gl.GPS_PVT
got = []
for v in ("false", "off", "no", "0", "auto", "1", "true", "maybe"):
    gl.GPS_PVT = None
    gl._gps_pvt_env({"GPS_PVT": v})
    got.append(gl.GPS_PVT)
gl.GPS_PVT = _pv
check("R2-PVT: GPS_PVT false/off/no/0 turn it off, auto/1/true on",
      got == [False, False, False, False, True, True, True, None], repr(got))

# R2-PORTAL: an HTML 2xx (a captive portal) does not count as sent
online(_r2_dir)
pp = os.path.join(_r2_dir, "20261101_130000.trk")
make_trk(pp, 3)
_post_headers[0] = {"Content-Type": "text/html; charset=utf-8"}
_post_code[0] = 200
diag_clear()
_dg = gl.DIAG
gl.DIAG = 60000
try:
    pump()
finally:
    gl.DIAG = _dg
portal = (os.path.exists(pp), gl.up["sent"],
          any("captive portal" in l for l in diag_lines()))
_post_headers[0] = {"Content-Type": "application/json"}
_post_code[0] = 201
gl.up["next_at"] = _clock[0]
pump()
check("R2-PORTAL: an HTML 200 keeps the track (not sent); a JSON 201 sends "
      "it", portal == (True, 0, True) and not os.path.exists(pp)
      and gl.up["sent"] == 3, repr((portal, gl.up["sent"])))
_post_headers[0] = {}
diag_clear()

# R2-HIST: the day history yields to the tracks: older days go (today's
# stays) when the flash runs low, also before a batch would be dropped
_hd2 = gl.HIST_DIR
gl.HIST_DIR = os.path.join(_r2_dir, "hist")
os.makedirs(gl.HIST_DIR, exist_ok=True)
for d in ("20261029.trk", "20261030.trb", "20261031.trk", "20261101.trb"):
    with open(os.path.join(gl.HIST_DIR, d), "w") as f:
        f.write("x" * 100)
_fr = gl.free
gl.free = lambda: 2000000 - 400000 * len(os.listdir(gl.HIST_DIR))
try:
    room = gl._hist_trim(1000000)
    left = sorted(os.listdir(gl.HIST_DIR))
    gl.free = lambda: 1000
    room2 = gl._hist_trim(10 ** 9)
    left2 = sorted(os.listdir(gl.HIST_DIR))
finally:
    gl.free = _fr
    gl.HIST_DIR = _hd2
check("R2-HIST: oldest days go until enough is free (.trk and .trb), "
      "today's file stays",
      left == ["20261031.trk", "20261101.trb"] and room == 1200000
      and left2 == ["20261101.trb"], repr((left, room, left2)))

# R2-RM: a file that cannot be deleted after its upload is parked as .err
# after READ_ERR_MAX tries (its last batch went up each time)
pr = os.path.join(_r2_dir, "20261101_140000.trk")
make_trk(pr, 2)
_rmr = os.remove
os.remove = lambda path, *a: (_ for _ in ()).throw(OSError(5)) \
    if path == pr else _rmr(path, *a)
res = []
try:
    for _ in range(gl.READ_ERR_MAX):
        try:
            gl._rm_done(pr)
            res.append("ok")
        except OSError:
            res.append("raised")
finally:
    os.remove = _rmr
check("R2-RM: raises READ_ERR_MAX-1 times, then parks the file as .err",
      res == ["raised"] * (gl.READ_ERR_MAX - 1) + ["ok"]
      and os.path.exists(pr + ".err") and not os.path.exists(pr),
      repr(res))

# R2-LOST: a radio turned off on purpose clears lost_at ("back after"
# measures a reconnect, not an hour of sleep)
class _OffW(object):
    def disconnect(self):
        pass
    def active(self, x=None):
        return False
gl.wifi_reset()
gl.log_reset()
gl.wifi["lost_at"] = _clock[0]
gl._wifi_off(_OffW(), "sleep")
check("R2-LOST: _wifi_off clears lost_at", gl.wifi["lost_at"] is None)

# R2-POLL: a failure status other than the one the clear left fails the try
# at once (the radio stayed on CONNECT_TIMEOUT for every absent network)
class _PollW(object):
    def __init__(self, s):
        self.s = s
    def status(self, arg=None):
        return self.s
gl.wifi_reset()
fails = []
_wf = gl._wifi_failed
gl._wifi_failed = lambda w, s, el: fails.append((s, el)) or False
try:
    gl.wifi.update(state="connecting", try_at=_clock[0], prev_status=0)
    _clock[0] += 500
    gl._wifi_poll(_PollW(201))
    fast = list(fails)
    gl.wifi.update(try_at=_clock[0], prev_status=201)
    _clock[0] += 500
    gl._wifi_poll(_PollW(201))
    held = list(fails)
finally:
    gl._wifi_failed = _wf
    gl.wifi_reset()
check("R2-POLL: a new failure status fails at once; the one the clear left "
      "waits for CONNECT_TIMEOUT",
      fast == [(201, 500)] and held == fast, repr((fast, held)))

# R2-WEB: a bug on a page request is caught (no reset), counted, closed
class _Cli(object):
    closed = False
    def settimeout(self, t):
        pass
    def close(self):
        _Cli.closed = True
class _Lsn(object):
    def accept(self):
        return _Cli(), ("1.2.3.4", 5)
_wsaved = (gl._web_sock, gl._web_read, gl._web_reply, gl.WEB)
gl._web_sock, gl.WEB = _Lsn(), True
gl._web_read = lambda c: b"GET / HTTP/1.0\r\n\r\n"
gl._web_reply = lambda c, r: (_ for _ in ()).throw(TypeError("bug"))
gl.wifi_reset()
gl.wifi["state"] = "connected"
gl._web_errs[0] = 0
try:
    gl.web_tick()
    r_web = "ok"
except Exception as e:
    r_web = repr(e)
finally:
    gl._web_sock, gl._web_read, gl._web_reply, gl.WEB = _wsaved
    gl.wifi_reset()
check("R2-WEB: a TypeError on a page request does not escape web_tick",
      r_web == "ok" and gl._web_errs[0] == 1 and _Cli.closed, r_web)
shutil.rmtree(_r2_dir, ignore_errors=True)

# --- web review (WEBR-n) ------------------------------------------------------
_wr_dir = tempfile.mkdtemp(prefix="wr-")
_wr_saved = (gl.DIAG, gl.DIAG_FILE, gl.DIAG_OLD, gl._web_rows, gl.WEB_PAGE)
gl.DIAG = 60000
gl.DIAG_FILE = os.path.join(_wr_dir, "diag.log")
gl.DIAG_OLD = os.path.join(_wr_dir, "diag.old")


def _wr_bugs(prefix):
    return [x for x in diag_lines() if x.startswith(prefix)]


# WEBR-2: status.json builds its rows before the 200: a bug in one is a 500
# with the error (it was a 200 with a cut JSON: the page said OFFLINE),
# counted in web_errs and logged once per kind (only the first error since
# boot reached diag)
def _wr_row_bug():
    raise TypeError("row bug")


gl._web_errs[0] = 0
gl._bugs[:] = []
gl.wifi_reset()
gl._web_rows = _wr_row_bug
c, c2 = RvConn(b""), RvConn(b"")
try:
    for c_ in (c, c2):
        try:
            gl._web_reply(c_, b"GET /status.json HTTP/1.1\r\n\r\n")
        except Exception as e:           # (web_tick's catch, after the 200)
            print("web error", repr(e))
finally:
    gl._web_rows = _wr_saved[3]
w2_head, _, w2_body = c.sent.partition(b"\r\n\r\n")
check("WEBR-2: a bug building status.json is a 500 with the error, not a "
      "200 with a cut JSON",
      w2_head.startswith(b"HTTP/1.0 500") and b"TypeError" in w2_body
      and b"row bug" in w2_body and c2.sent.startswith(b"HTTP/1.0 500"),
      repr(c.sent[:200]))
_wsaved = (gl._web_sock, gl._web_read, gl._web_reply, gl.WEB)
gl._web_sock, gl.WEB = _Lsn(), True
gl._web_read = lambda c: b"GET / HTTP/1.0\r\n\r\n"
gl.wifi["state"] = "connected"
try:
    for e_ in (KeyError("k"), KeyError("k"), ValueError("v")):
        gl._web_reply = lambda c, r, e_=e_: (_ for _ in ()).throw(e_)
        gl.web_tick()
finally:
    gl._web_sock, gl._web_read, gl._web_reply, gl.WEB = _wsaved
    gl.wifi_reset()
try:
    w2_errs = json.loads(gl._web_json()).get("web_errs")
except ValueError:
    w2_errs = "bad json"
check("WEBR-2: every new kind of web error reaches diag once (a repeat does "
      "not); status.json has web_errs",
      _wr_bugs("web error") == ["web error TypeError('row bug')",
                                "web error KeyError('k')",
                                "web error ValueError('v')"]
      and gl._web_errs[0] == 5 and w2_errs == 5,
      repr((_wr_bugs("web error"), gl._web_errs[0], w2_errs)))
gl._web_errs[0] = 0

# WEBR-5: the DNS-rebinding guard matches Host in any case, a bare-LF head
# too, and a head cut at WEB_REQ_MAX (its Host maybe after the cut) is no
# "no Host"
gl.wifi["ip"] = "192.168.1.40"
w5_hn = gl.HOSTNAME
w5 = [gl._web_host_ok(r) for r in (
    b"GET / HTTP/1.1\r\nHOST: evil.example\r\n\r\n",
    b"GET / HTTP/1.1\r\nhOsT: evil.example\r\n\r\n",
    b"GET / HTTP/1.1\nHost: evil.example\n\n",
    gl._web_read(RvConn(b"GET /" + b"a" * 2600
                        + b" HTTP/1.1\r\nHost: evil.example\r\n\r\n")),
    b"GET / HTTP/1.1\r\nHost: 192.168.1.4",
    b"GET / HTTP/1.1\r\nHOST: " + w5_hn.upper().encode() + b".LOCAL:80\r\n\r\n",
    b"GET / HTTP/1.1\nhost: 192.168.1.40\n\n",
    b"GET / HTTP/1.0\r\nUser-Agent: curl\r\n\r\n",
    b"GET /status.json\r\n")]
gl.wifi_reset()
check("WEBR-5: Host any case, bare LF, a cut head: a foreign name or an "
      "unknown Host is refused; the board's names and no Host pass",
      w5 == [False, False, False, False, False, True, True, True, True],
      repr(w5))


# WEBR-6: a bare-LF head (\n\n) and an HTTP/0.9 request line end the head:
# they waited for more until the timeout and got nothing
class RvHold(RvConn):
    """A client that sent its request and waits: recv times out."""
    def recv(self, n):
        if not self.req:
            raise OSError(110)
        return RvConn.recv(self, n)


w6 = []
for r_ in (b"GET /status.json HTTP/1.0\n\n", b"GET /status.json\r\n",
           b"GET /\n", b"GET / HTTP/1.1\r\nHost: x\r\n\r\n"):
    try:
        w6.append(gl._web_read(RvHold(r_)) == r_)
    except OSError as e:
        w6.append(repr(e))
try:
    gl._web_read(RvHold(b"GET / HTTP/1.1\r\nHost: x\r\n"))
    w6.append("no wait")
except OSError:
    w6.append("waits")
gl.WEB_PAGE = os.path.join(ROOT, "web.html")
c = RvConn(b"")
gl._web_reply(c, b"GET /\r\n")
c_lf = RvConn(b"")
gl._web_reply(c_lf, b"GET / HTTP/1.0\nAccept: */*\n\n")
gl.WEB_PAGE = _wr_saved[4]
check("WEBR-6: a bare-LF or HTTP/0.9 head is read without waiting for more "
      "and answered; an unfinished head still waits",
      w6 == [True, True, True, True, "waits"]
      and c.sent.startswith(b"HTTP/1.0 200 OK")
      and b"<html" in c.sent[:2000]
      and c_lf.sent.startswith(b"HTTP/1.0 200 OK"),
      repr((w6, c.sent[:60], c_lf.sent[:60])))

# WEBR-9: a bug in the upload, OTA or WiFi tick is logged (once a kind) and
# the loop goes on, the tick paused WR_PAUSE (no hot retry), after a
# MemoryError a gc.collect: it escaped into run()'s crash branch, a reset
# and a boot-fail count, and a persistent one left the board in the REPL
# after BOOT_FAIL_MAX boots, logging nothing. log_tick is not guarded,
# nor is anything while an OTA image is pending (its crash rolls it back)
_w9_saved = (gl.upload_tick, gl.ota_tick, gl.wifi_tick, gl.log_tick,
             gl.time.sleep_ms, gl.gc.collect, gl._wdt, gl._boot_ok)
w9 = {"log": 0, "up": 0, "ota": 0, "wifi": 0, "mem": 0, "gc": 0}


def _w9_up():
    w9["up"] += 1
    raise ValueError("bad batch")


def _w9_ota(now):
    w9["ota"] += 1
    w9["mem"] += 1
    raise MemoryError()


def _w9_wifi():
    w9["wifi"] += 1
    raise TypeError("wifi bug")


def _w9_log():
    w9["log"] += 1


def _w9_gc():
    if w9["mem"]:
        w9["mem"] -= 1
        w9["gc"] += 1
    return real_gc.collect()


_w9_n = [0, 600]


def _w9_sleep(ms):
    _clock[0] += ms
    _w9_n[0] += 1
    if _w9_n[0] >= _w9_n[1]:
        raise KeyboardInterrupt


def _w9_run(n):
    _w9_n[:] = [0, n]
    gl._wdt = None
    try:
        gl._loop()
        return "returned"
    except KeyboardInterrupt:
        return "ki"
    except Exception as e:
        return repr(e)


gl.upload_tick, gl.ota_tick, gl.wifi_tick = _w9_up, _w9_ota, _w9_wifi
gl.log_tick, gl.time.sleep_ms, gl.gc.collect = _w9_log, _w9_sleep, _w9_gc
gl._boot_ok = lambda: None
gl.wifi_reset()
gl.ota["pend"] = None
gl._tick_bug.clear()
diag_clear()
try:
    w9_r = _w9_run(600)                  # 12 s of loop
    w9_calls = dict(w9)
    w9_fed = gl._wdt.fed if gl._wdt is not None else 0
    _clock[0] += 10000                   # past the pause
    gl.wifi_tick = lambda: None
    gl.ota["pend"] = 123                 # a new image runs pending
    w9_pend = _w9_run(5)
    gl.ota["pend"] = None
    gl.upload_tick = lambda: None
    gl.log_tick = lambda: (_ for _ in ()).throw(RuntimeError("log bug"))
    w9_log = _w9_run(5)
finally:
    (gl.upload_tick, gl.ota_tick, gl.wifi_tick, gl.log_tick,
     gl.time.sleep_ms, gl.gc.collect, gl._wdt, gl._boot_ok) = _w9_saved
    gl.ota["pend"] = None
    gl.wifi_reset()
check("WEBR-9: an upload / OTA / WiFi tick that raises: the loop goes on "
      "(the WDT fed), one diag line a kind, the tick paused (not every "
      "20 ms), gc.collect after a MemoryError",
      w9_r == "ki" and w9_calls["log"] == 600 and w9_fed >= 600
      and all(2 <= w9_calls[k] <= 4 for k in ("up", "ota", "wifi"))
      and w9_calls["gc"] == w9_calls["ota"]
      and sorted(_wr_bugs("bug ")) == ["bug ota MemoryError()",
                                      "bug upload ValueError('bad batch')",
                                      "bug wifi TypeError('wifi bug')"],
      repr((w9_r, w9_calls, w9_fed, _wr_bugs("bug "))))
check("WEBR-9: a pending OTA image and a log_tick bug still crash (run()'s "
      "branch resets; the bootloader rolls the image back)",
      w9_pend == "ValueError('bad batch')"
      and w9_log == "RuntimeError('log bug')", repr((w9_pend, w9_log)))
# GUARD-INT: the guard against the other fixes. (1) an upload bug after the
# POST, before the commit: the guard swallowed it with nothing scheduled,
# the same batch went up again every TICK_PAUSE, its points into the day
# history each time; the upload is now backed off (_schedule) and the
# history copy is best effort (any error). (2) a tick that ends the
# pending state and then raises (the OTA health reset's own path): the
# guard read ota["pend"] after the call, None, and swallowed it: no reset,
# the image ran unconfirmed. (3) a MemoryError that keeps coming: the
# crash reset was its recovery; GUARD_MEM_MAX in a row are a memory reboot
gi_saved = (gl._mem_reboot, gl._mem_reboot_soon, gl._track_lines,
            gl.HIST_DAYS)
gi_reb = []
gl._mem_reboot = lambda why: gi_reb.append(why)
gl._mem_reboot_soon = lambda: False
try:
    gl._tick_bug.clear()
    gl.ota["pend"] = None
    gl.up["next_at"], gl.up["backoff"] = None, None
    gl._guarded("upload", lambda: (_ for _ in ()).throw(ValueError("x")))
    gi_sched = gl.up["next_at"] is not None and gl._waiting(
        _clock[0], gl.up["next_at"], gl.UP_WAIT_MAX)
    gl.HIST_DAYS = 7
    gl._track_lines = lambda *a: (_ for _ in ()).throw(TypeError("y"))
    gi_tr = os.path.join(tempfile.mkdtemp(prefix="gi-"), "20261102_100000.trb")
    with open(gi_tr, "wb") as f:
        f.write(b"x")
    try:
        gl._hist_batch(gi_tr, 0, 1, 0)
        gi_hist = "ok"
    except Exception as e:
        gi_hist = repr(e)

    def _gi_end():
        gl.ota["pend"] = None            # (as _ota_health does)
        raise MemoryError()
    gl._tick_bug.clear()
    gl.ota["pend"] = 123
    try:
        gl._guarded("ota", _gi_end)
        gi_pend = "swallowed"
    except MemoryError:
        gi_pend = "raised"
    gl.ota["pend"] = None
    for _ in range(gl.GUARD_MEM_MAX):
        gl._tick_bug.clear()
        gl._guarded("wifi", lambda: (_ for _ in ()).throw(MemoryError()))
    gi_mem = list(gi_reb)
finally:
    (gl._mem_reboot, gl._mem_reboot_soon, gl._track_lines,
     gl.HIST_DAYS) = gi_saved
    gl._tick_bug.clear()
    gl.up["next_at"], gl.up["backoff"] = None, None
check("GUARD-INT: an upload bug backs the upload off, the history copy "
      "takes any error, a tick that ends the pending state still raises, "
      "GUARD_MEM_MAX MemoryErrors in a row reboot",
      gi_sched and gi_hist == "ok" and gi_pend == "raised"
      and len(gi_mem) == 1, repr((gi_sched, gi_hist, gi_pend, gi_mem)))
# WEB-JSON-TAIL: an error after the 200 (reading the diag tail, a
# satellite) still cut the JSON (the page: OFFLINE). The parts after the
# rows end the JSON instead, the error counted
wj_saved = gl._web_diag


def _wj_diag():
    yield "first line"
    raise OSError(5)


gl._web_diag = _wj_diag
try:
    wj_text = "".join(gl._web_json_parts())
    try:
        wj = json.loads(wj_text)
        wj_ok = (wj["diag"] == ["first line"] and "satellites" in wj)
    except ValueError as e:
        wj_ok = repr(e)
finally:
    gl._web_diag = wj_saved
check("WEB-JSON-TAIL: an error while streaming the diag lines still ends "
      "a valid JSON", wj_ok is True, repr((wj_ok, wj_text[-120:])))
(gl.DIAG, gl.DIAG_FILE, gl.DIAG_OLD, gl._web_rows, gl.WEB_PAGE) = _wr_saved
shutil.rmtree(_wr_dir, ignore_errors=True)

# R2-UPTIME: uptime counts from boot (ticks at import), not from the import
gl = _reload_logger()
check("R2-UPTIME: _up starts at the ticks of the import (boot), not at 0",
      gl._up[0] * 1000 + gl._up[2] == gl._up[1] and 0 <= gl._up[2] < 1000,
      repr(list(gl._up)))

# OTA-RETRY: checks every OTA_CHECK (10 min); a download that failed its
# sha256 is not downloaded again before OTA_RETRY; one diag line per kind
# of error (a server down every 10 min filled the log)
_ota_dir2 = tempfile.mkdtemp()
_or_saved = (gl.esp32, gl.HTTP_GET, gl.OTA_URL, gl.OTA_KEY, gl.FW_BUILD,
             gl.FW_COMMIT, gl.OTA_PENDING, gl.OTA_BAD, gl.machine.reset,
             gl.DIAG, gl.OTA_TRIES, gl.OTA_PAGES)
_ota_dir = _ota_dir2
gl.esp32, gl.HTTP_GET = _FakeEsp32, _ota_get
gl.OTA_URL, gl.OTA_KEY = "http://ota.lan/gps/manifest.json", b"secret"
gl.FW_BUILD, gl.FW_COMMIT = 100, "abc0100"
gl.OTA_PENDING = os.path.join(_ota_dir2, "ota.pending")
gl.OTA_BAD = os.path.join(_ota_dir2, "ota.bad")
gl.OTA_TRIES = os.path.join(_ota_dir2, "ota.tries")
gl.OTA_PAGES = (os.path.join(_ota_dir2, "web.html.gz"),)
gl.OTA_MARK = os.path.join(_ota_dir2, "ota.mark")
gl.OTA_SHADOW = (os.path.join(_ota_dir2, "gps_logger.mpy"),)
gl.machine.reset = lambda: _ota_log.append(("reset",))
gl.DIAG = 60000
try:
    _ota_reset()
    gl.ota["failed"] = None
    _serve(_manifest(200), img=_img[:-1] + b"x")
    gl._ota_check()
    w1 = _slots[1].writes
    s1 = gl.ota["state"]
    _clock[0] += gl.OTA_CHECK
    gl._ota_check()
    w2 = _slots[1].writes
    _clock[0] += gl.OTA_RETRY
    _serve(_manifest(200))
    gl._ota_check()
    retried = (_slots[1].writes > w2, list(_ota_log))
    # errors: the same one twice is one diag line, a new one another
    _ota_reset()
    gl.ota.update(last_err=None, failed=None)
    gl.wifi_reset()
    gl.log_reset()
    gl.up_reset()
    gl.wifi.update(state="connected", home=True)
    def _boom():
        raise OSError(113)
    _sc = gl._ota_check
    gl._ota_check = _boom
    try:
        for k in range(3):
            gl.ota["at"] = _clock[0] - gl.OTA_CHECK
            gl.ota_tick(_clock[0])
    finally:
        gl._ota_check = _sc
    errs = [l for l in diag_lines() if l.startswith("ota error")]
finally:
    (gl.esp32, gl.HTTP_GET, gl.OTA_URL, gl.OTA_KEY, gl.FW_BUILD,
     gl.FW_COMMIT, gl.OTA_PENDING, gl.OTA_BAD, gl.machine.reset,
     gl.DIAG, gl.OTA_TRIES, gl.OTA_PAGES) = _or_saved
    gl.wifi_reset()
    gl.log_reset()
    diag_clear()
    shutil.rmtree(_ota_dir2, ignore_errors=True)
check("OTA-RETRY: OTA_CHECK is 10 min; a bad download waits OTA_RETRY "
      "before the next try; the same error is one diag line",
      gl.OTA_CHECK == 600000 and s1 == "bad image" and w2 == w1
      and retried == (True, [("boot", "ota_1"), ("reset",)])
      and errs == ["ota error OSError(113)"],
      repr((s1, w1, w2, retried, errs)))

# --- TRKB: binary delta tracks (.trb) ----------------------------------------
# The same text lines, each stored as its difference from the line before:
# the upload JSON, the .bad copy and the day history are the text lines
# byte for byte; a point takes ~2-8 bytes instead of ~46-51.
import random as _rnd
gl.reset()
_tb_saved = (gl.LOG_DIR, gl.HIST_DIR, gl.UPLOAD_BATCH)
_tb = tempfile.mkdtemp(prefix="trb-")
gl.HIST_DIR = os.path.join(_tb, "hist")


def _tb_drive(n, fmt, seed=1, lat=47.185356, lon=18.426934, step=1,
              gaps=True):
    """n text lines as _line writes them: a 1 Hz drive (fmt 2: NAV-PVT
    ints, acc and speed; fmt 1: NMEA floats and hdop)."""
    r = _rnd.Random(seed)
    out = []
    t = 1793520000 - gl.EPOCH_DAYS * 86400     # 2026-11-01
    li, lo = int(lat * 1e7), int(lon * 1e7)
    ele, hd, spd, head = 120.0, 1.2, 13.0, 0.4
    for i in range(n):
        t += step if r.random() > 0.05 else r.randint(2, 40)
        head += r.uniform(-0.08, 0.08)
        spd = max(0.0, min(36.0, spd + r.uniform(-0.6, 0.6)))
        li += int(spd * 90 * math.cos(head))   # ~1e-7 deg per 1.1 cm
        lo += int(spd * 133 * math.sin(head))
        ele += r.uniform(-0.3, 0.3)
        if r.random() < 0.1:
            hd = round(max(0.6, min(4.0, hd + r.uniform(-0.3, 0.3))), 1)
        tc = gl._stamp_s(t)[1]
        e = "%.1f" % ele if not gaps or r.random() > 0.01 else ""
        if fmt == 2:
            c = gl._i7(li) + "," + gl._i7(lo)
            out.append("%s,%s,%d,%s,%s\n" % (c, e, max(1, int(hd * 2)),
                                             "%.1f" % spd, tc))
        else:
            c = "%.6f,%.6f" % (li / 1e7, lo / 1e7)
            out.append("%s,%s,%s,%s\n" % (c, e, "%.1f" % hd, tc))
    return out


def _tb_write(path, lines, per=60, fmt=None):
    """lines through flush() into path, per lines a flush (as the logger
    writes a drive: FLUSH every 60 s)."""
    gl.log_reset()
    gl.log["file"] = path
    gl.log["fmt"] = fmt or (2 if lines[0].count(",") == 5 else 1)
    for i in range(0, len(lines), per):
        gl.log["batch"] = list(lines[i:i + per])
        gl.flush()
    gl.log["file"] = None


def _tb_bodies(path):
    """Every read_batch body of path, offset by offset as the upload goes."""
    out, off, key = [], 0, None
    for _ in range(10000):
        n, ok, size, new_off, eof = gl.read_batch(path, off, key)
        out.append((n, ok, bytes(gl._body[:size])))
        key = gl._rk[0]
        if eof:
            break
        off = new_off
    return out


def _tb_keys(path):
    """The records of a .trb in order: "=" keyframe, "d" delta, "-", "#"."""
    out = []
    with builtins.open(path, "rb") as f:
        gl._rd_open(0)
        while gl._rd_rec(f):
            c = gl._rbuf[gl._rd[1]]
            out.append("d" if c >= 128 else chr(c))
    return "".join(out)


def _tb_raw(path):
    with builtins.open(path, "rb") as f:
        return f.read()


try:
    gl.LOG_DIR = _tb
    sizes = {}
    same = {}
    for fmt in (1, 2):
        lines = _tb_drive(1500, fmt, seed=fmt)
        pt = os.path.join(_tb, "20261101_08000%d.trk" % fmt)
        pb = os.path.join(_tb, "20261101_08000%d.trb" % fmt)
        _tb_write(pt, lines)
        _tb_write(pb, lines)
        raw = _tb_raw(pb)
        sizes[fmt] = (len(raw) - len(gl.HEADERB) - len(gl.FOOTER)) / len(lines)
        dec = list(trk2gpx.trb_lines(raw))
        same[fmt] = (dec[1:-1] == lines, _tb_bodies(pt) == _tb_bodies(pb),
                     gl.count_points() if False else None)
        rs = _tb_keys(pb)
        same[fmt] += (rs.startswith("#=") and rs.endswith("d#"),
                      rs.count("="), os.path.getsize(pt) / len(lines))
        pk = os.path.join(_tb, "20261101_08100%d.trb" % fmt)
        _tb_write(pk, _tb_drive(400, fmt, seed=fmt, gaps=False), per=400)
        same[fmt] += (sum(1 for a_, b_ in zip(lines, lines[1:])
                          if (",," in a_) != (",," in b_)),)
        rk = _tb_keys(pk)[1:-1]
        same[fmt] += (rk == ("=" + "d" * gl.KEY_EVERY) * 6 + "=" + "d" * 9,)
    print("TRKB bytes/point", sizes, "text", same[1][5], same[2][5])
    g_t = trk2gpx.convert(open(pt), "x")
    g_b = trk2gpx.convert(trk2gpx.trb_lines(_tb_raw(pb)), "x")
    _so = sys.stdout
    sys.stdout = io.StringIO()
    try:
        rc_ = trk2gpx.main(["trk2gpx.py", pb])
        g_m = sys.stdout.getvalue()
    finally:
        sys.stdout = _so
    check("TRKB: trk2gpx converts a .trb to the GPX of its .trk",
          g_b == g_t and rc_ == 0 and g_m == g_b.replace(
              "<name>x</name>", "<name>20261101_080002</name>")
          and g_t.count("<trkpt") == 1500, repr((rc_, len(g_b), len(g_t))))
    check("TRKB: a flushed drive decodes to the very text lines (trk2gpx), "
          "TRK1 and TRK2",
          same[1][0] and same[2][0])
    check("TRKB: read_batch sends byte-identical Overland JSON, batch for "
          "batch, as from the .trk",
          same[1][1] and same[2][1])
    check("TRKB: under 10 bytes a point on a 1 Hz drive (TRK1 ~46, TRK2 ~51)",
          sizes[1] < 10 and sizes[2] < 10, repr(sizes))
    check("TRKB: each flush starts with a keyframe, then one every "
          "KEY_EVERY deltas",
          same[1][3] and same[2][3] and same[1][7] and same[2][7]
          and 25 <= same[1][4] <= 25 + same[1][6]
          and 25 <= same[2][4] <= 25 + same[2][6],
          repr((same[1][3:], same[2][3:])))

    # lines no delta can stand for are keyframes, kept as written
    odd = ["47.500000,19.000000,100.0,1.2,20261101T090000Z\n",
           "47.500010,19.000000,-0.0,1.2,20261101T090001Z\n",   # -0.0
           "47.500020,19.000000,05,1.2,20261101T090002Z\n",     # 05
           "47.500030,19.000000,1e3,1.2,20261101T090003Z\n",    # 1e3
           "47.500040,19.000000,100.0,1.2,20261101T090003Z\n",  # same s
           "47.500050,19.000000,100.0,1.2,20261101T090002Z\n",  # back
           "47.50006,19.000000,100.0,1.2,20261101T090005Z\n",   # decimals
           "47.500070,19.000000,,1.2,20261101T090006Z\n",       # no ele
           "47.500080,19.000000,,,20261101T090007Z\n",          # no hdop
           "47.500090,19.000000,,,20261101T090008Z\n",
           "47.500100,19.000000,100.0,1.2,4,20261101T090009Z\n",  # TRK2
           "47.500110,19.000000,100.0,1.2,4,20261101T090010Z\n",
           "-47.500110,-19.000001,-12.5,3,0.0,20261101T090011Z\n",
           "-47.500100,-19.000002,-12.4,3,0.0,20261101T090012Z\n",
           "-0.000001,0.000000,0.0,3,0.0,20261101T090013Z\n",
           "0.000001,-0.000001,0.0,3,0.0,20261101T090014Z\n",
           "47.5,19.0,1.0,2,1.0,20261131T000000Z\n",           # 31 Nov
           "47.5,19.0,1.0,2,1.0,20261130T000000Z\n",
           "47.5,19.0,1.0,2,1.0,20281231T235959Z\n",           # far
           "47.5,19.0,1.0,2,1.0,20290101T000000Z\n",
           "179.999999,179.999999,9999.9,999,99.9,20290101T000001Z\n"]
    po = os.path.join(_tb, "20261101_090000.trb")
    pot = os.path.join(_tb, "20261101_090000.trk")
    _tb_write(po, odd, per=7, fmt=2)
    _tb_write(pot, odd, per=7, fmt=2)
    raw = _tb_raw(po)
    dec = list(trk2gpx.trb_lines(raw))
    nkey = _tb_keys(po).count("=")
    check("TRKB: odd lines (-0.0, 05, 1e3, a second twice or back, other "
          "decimals, empty fields, TRK1/TRK2 mixed, negative, a day that "
          "does not exist) round-trip as keyframes",
          dec[1:-1] == odd and _tb_bodies(po) == _tb_bodies(pot)
          and nkey >= 12, repr((nkey, [(a, b) for a, b in zip(dec[1:-1], odd)
                                      if a != b][:2])))
    rec = [0, 0]
    g = gl._ks
    gl._kb_parse(odd[0].encode(), 0, len(odd[0]) - 1, g)
    chk = (g[0] is not None and g[1] == 47500000 and g[6] == 6
           and g[3] == 1000 and g[8] == 1 and g[11] == 4)
    for bad_ in ("05", "-0", "-0.0", "1.", ".5", "1e3", "+1", "1234567890",
                 "nan", " 1", "1-"):
        ln = ("47.5,19.0,%s,1.2,20261101T090000Z" % bad_).encode()
        gl._kb_parse(ln, 0, len(ln), g)
        rec[0] += g[0] is None
    for ok_ in ("0", "-1", "0.5", "-0.5", "123456789", "-12.34"):
        ln = ("47.5,19.0,%s,1.2,20261101T090000Z" % ok_).encode()
        gl._kb_parse(ln, 0, len(ln), g)
        rec[1] += g[0] is not None
    check("TRKB: only numbers written as they print are deltas (no -0, "
          "leading zero, exponent, over 9 digits)",
          chk and rec == [11, 6], repr((chk, rec)))

    # segment breaks stay "-" lines, before the line recorded after them
    sl = _tb_drive(30, 2, seed=5)
    ps = os.path.join(_tb, "20261101_100000.trb")
    gl.log_reset()
    gl.log["file"] = ps
    gl.log["fmt"] = 2
    gl.log["batch"] = list(sl[:20])
    gl.log["new_seg"] = True
    gl.log["seg_tc"] = [sl[12][-17:-1]]
    gl.flush()
    gl.log["batch"] = list(sl[20:])
    gl.log["new_seg"] = True
    gl.log["seg_tc"] = [sl[20][-17:-1]]
    gl.flush()
    dec = list(trk2gpx.trb_lines(_tb_raw(ps)))
    check("TRKB: segment breaks are '-' lines where the text has them",
          dec[1:-1] == sl[:12] + ["-\n"] + sl[12:20] + ["-\n"] + sl[20:],
          repr(dec[10:16]))

    # the upload: resumed from the .pos (offset and keyframe) after every
    # batch, across a reboot each time; each point up once, in order
    online(_tb)
    gl.LOG_DIR = _tb
    rl = _tb_drive(400, 2, seed=7)
    pr = os.path.join(_tb, "20261101_110000.trb")
    _tb_write(pr, rl)
    left0 = gl.count_points()
    poss = []
    for _ in range(20):
        if not os.path.exists(pr):
            break
        pump()
        if os.path.exists(pr + ".pos"):
            with builtins.open(pr + ".pos") as f:
                poss.append(f.read())
        gl.up_reset()                    # a reboot: the .pos only
        gl.up["url"] = "https://dawarich.example.com"
        gl.up["key"] = "secret-key"
    sent = locations()
    want = []
    for ln in rl:
        f_ = ln[:-1].split(",")
        want.append((round(float(f_[1]), 6), round(float(f_[0]), 6),
                     "%s-%s-%sT%s:%s:%sZ" % (f_[5][0:4], f_[5][4:6],
                                             f_[5][6:8], f_[5][9:11],
                                             f_[5][11:13], f_[5][13:15])))
    got = [(x["geometry"]["coordinates"][0], x["geometry"]["coordinates"][1],
            x["properties"]["timestamp"]) for x in sent]
    check("TRKB: uploaded in batches across reboots from the .pos "
          "'offset keyframe': every point once, in order",
          got == want and left0 == 400 and len(_posts) == 8
          and not os.path.exists(pr) and not os.path.exists(pr + ".pos")
          and all(len(x.split()) == 2 for x in poss),
          repr((len(got), len(want), left0, len(_posts), poss[:2])))

    # a .pos without (or with a wrong) keyframe resends from 0: nothing lost
    for posv in ("%d\n", "%d 3\n", "%d 99999999\n", "%d -2\n"):
        online(_tb)
        gl.LOG_DIR = _tb
        _tb_write(pr, rl[:120])
        n, ok, size, off1, eof = gl.read_batch(pr, 0)
        k1 = gl._rk[0]
        with builtins.open(pr + ".pos", "w") as f:
            f.write(posv % off1 if "%" in posv else posv)
        o2, k2 = gl.read_pos(pr)
        for _ in range(10):
            if os.path.exists(pr):
                pump()
        ts = [x["properties"]["timestamp"] for x in locations()]
        check("TRKB: a .pos %r resends from the start, nothing lost"
              % posv.strip(),
              (o2, k2) == (0, None) and len(set(ts)) == 120
              and k1 is not None, repr((o2, k2, len(ts), len(set(ts)))))

    # UPL-8: a .pos "offset -1" (no keyframe before it) is checked from the
    # start of the file: a corrupt one inside a record resends from 0 (it
    # was read from there, misframed); at a record start the records
    # before it are read again for the state, nothing lost
    u8 = []
    for d_ in (1, 0):
        online(_tb)
        gl.LOG_DIR = _tb
        _tb_write(pr, rl[:120])
        n, ok, size, off1, eof = gl.read_batch(pr, 0)
        with builtins.open(pr + ".pos", "w") as f:
            f.write("%d -1\n" % (off1 + d_))
        left_ = gl.count_points()
        for _ in range(10):
            if os.path.exists(pr):
                pump()
        ts = [x["properties"]["timestamp"] for x in locations()]
        u8.append((left_, len(ts), len(set(ts))))
    check("UPL-8: a .pos 'offset -1' inside a record resends from 0; at a "
          "record start the rest goes up whole",
          u8 == [(120, 120, 120), (70, 70, 70)], repr(u8))

    # a misaligned offset (not on a record) with a good keyframe: from 0
    online(_tb)
    gl.LOG_DIR = _tb
    _tb_write(pr, rl[:120])
    n, ok, size, off1, eof = gl.read_batch(pr, 0)
    k1 = gl._rk[0]
    gl.write_offset(pr, off1 + 1, k1)
    for _ in range(10):
        if os.path.exists(pr):
            pump()
    ts = [x["properties"]["timestamp"] for x in locations()]
    check("TRKB: an offset inside a record resends from 0",
          len(set(ts)) == 120, repr(len(ts)))

    # count_points counts the records left from the .pos
    online(_tb)
    gl.LOG_DIR = _tb
    _tb_write(pr, rl[:130])
    gl.UPLOAD_BATCH = 50
    n, ok, size, off1, eof = gl.read_batch(pr, 0)
    gl.commit_offset(pr, n, off1, eof, gl._rk[0])
    check("TRKB: count_points counts the points after the .pos",
          gl.count_points() == 80, repr(gl.count_points()))

    # count_points from a .pos whose offset is inside a record (its
    # keyframe good): counts the file from the start, as read_batch then
    # reads it (it counted misframed bytes: a wrong 'left' held the OTA)
    online(_tb)
    gl.LOG_DIR = _tb
    _tb_write(pr, rl[:130])
    n, ok, size, off1, eof = gl.read_batch(pr, 0)
    gl.write_offset(pr, off1 + 1, gl._rk[0])
    check("TRKB: count_points from an offset inside a record counts the "
          "whole file", gl.count_points() == 130, repr(gl.count_points()))

    # the encoder and the decoder share the time range: lines dated 2088
    # or later (past FIX_S_END) are keyframes, all read back (the encoder
    # wrote deltas the decoder dropped)
    far = ["47.5000%02d,19.000000,1.0,2,1.0,20900101T0000%02dZ\n" % (i, i)
           for i in range(6)]
    pf = os.path.join(_tb, "20900101_000000.trb")
    with builtins.open(pf, "wb") as f:
        f.write(gl.HEADERB.encode() + gl._kb_encode(far, len(far), None)
                + gl.FOOTER.encode())
    n_f, ok_f = gl.read_batch(pf, 0)[:2]
    check("TRKB: lines past FIX_S_END are keyframes, every one read back",
          (n_f, ok_f) == (6, 6) and _tb_keys(pf).count("=") == 6,
          repr((n_f, ok_f, _tb_keys(pf))))

    # 422: the batch goes to <file>.trb.bad as text lines, read as a .trk
    online(_tb)
    gl.LOG_DIR = _tb
    _tb_write(pr, rl[:70])
    _post_code[0] = 422
    pump()
    _post_code[0] = 201
    bad = pr + ".bad"
    with builtins.open(bad) as f:
        btxt = f.read()
    hist_q = []
    for d in os.listdir(gl.HIST_DIR):
        with open(os.path.join(gl.HIST_DIR, d)) as f:   # (a .trb as text)
            hist_q.append(f.read())
    check("TRKB: a rejected batch is parked as text lines in .trb.bad "
          "(listed as a .bad, read as text)",
          btxt == "".join(rl[:50]) and gl.pending_bad() == [bad]
          and gl._is_trk(bad) and not bad.endswith(gl.TRB)
          and "".join(rl[:50]) in "".join(hist_q),
          repr(btxt[:80]))
    for _ in range(10):
        pump()
    ts = [x["properties"]["timestamp"] for x in locations()]
    check("TRKB: ... the rest of the track and then the .bad go up",
          not os.path.exists(pr) and not os.path.exists(bad)
          and len(set(ts)) == 70, repr((len(ts), os.listdir(_tb))))

    # a file cut off mid-record (a damaged end): the points before it go up
    online(_tb)
    gl.LOG_DIR = _tb
    _tb_write(pr, rl[:90], per=90)
    raw = _tb_raw(pr)
    with builtins.open(pr, "wb") as f:
        f.write(raw[:len(raw) - len(gl.FOOTER) - 1])
    for _ in range(10):
        if os.path.exists(pr):
            pump()
    ts = [x["properties"]["timestamp"] for x in locations()]
    check("TRKB: a file cut off inside its last record uploads the points "
          "before it",
          len(ts) == 89 and not os.path.exists(pr), repr(len(ts)))

    # a damaged delta drops points only up to the next keyframe
    online(_tb)
    gl.LOG_DIR = _tb
    _tb_write(pr, rl[:120], per=60)
    raw = bytearray(_tb_raw(pr))
    i = raw.index(b"\n", 7) + 1          # the first delta record
    raw[i] = 0xC1                        # bit 6: no such field
    with builtins.open(pr, "wb") as f:
        f.write(raw)
    for _ in range(10):
        if os.path.exists(pr):
            pump()
    ts = [x["properties"]["timestamp"] for x in locations()]
    want = [x[2] for x in [(0, 0, "%s-%s-%sT%s:%s:%sZ" % (
        l[-17:-13], l[-13:-11], l[-11:-9], l[-8:-6], l[-6:-4], l[-4:-2]))
        for l in rl[:120]]]
    check("TRKB: a damaged record loses the points up to the next "
          "keyframe, none after it, no wrong ones",
          ts == [want[0]] + want[60:], repr((len(ts), ts[:2])))

    # the end of a .trb not where the last write left it: no guessing at a
    # record boundary, the batch goes on in a new file
    online(_tb)
    gl.LOG_DIR = _tb
    pw = os.path.join(_tb, "20261101_120000.trb")
    gl.log_reset()
    gl.log["file"] = pw
    gl.log["fmt"] = 2
    gl.log["batch"] = list(rl[:30])
    gl.flush()
    raw = _tb_raw(pw)
    with builtins.open(pw, "wb") as f:
        f.write(raw[:-len(gl.FOOTER)] + b"\x83\x05\n\x01")   # no footer
    gl.log["good_size"] = None
    gl.log["batch"] = list(rl[30:40])
    gl.flush()
    gl.log["batch"] = list(rl[40:50]) if gl.log["batch"] == [] else \
        gl.log["batch"]
    nf = gl.log["file"]
    if gl.log["batch"]:
        gl.flush()
    check("TRKB: a .trb whose footer is gone is not appended to: the "
          "batch goes to a new file",
          nf != pw and _tb_raw(pw) == raw[:-len(gl.FOOTER)] + b"\x83\x05\n\x01"
          and nf.endswith(gl.TRB) and gl.log["batch"] == []
          and list(trk2gpx.trb_lines(_tb_raw(nf)))[1:-1]
          == rl[30:30 + gl.log["points"]],
          repr((nf, gl.log["points"], len(gl.log["batch"]))))
    gl.log_reset()

    # no keyframe in the first batch (its "=" damaged): the upload goes on
    # from the .pos "offset -1", it does not send the same batch forever
    online(_tb)
    gl.LOG_DIR = _tb
    _tb_write(pr, rl[:120], per=120)
    raw = bytearray(_tb_raw(pr))
    raw[len(gl.HEADERB)] = 0x3E          # "=" -> ">"
    with builtins.open(pr, "wb") as f:
        f.write(raw)
    poss = []
    for _ in range(6):
        if os.path.exists(pr):
            pump()
            if os.path.exists(pr + ".pos"):
                with builtins.open(pr + ".pos") as f:
                    poss.append(f.read().split())
    ts = [x["properties"]["timestamp"] for x in locations()]
    check("TRKB: a first batch without a keyframe is passed ('offset -1'), "
          "the rest goes up once",
          not os.path.exists(pr) and poss and poss[0][1] == "-1"
          and len(_posts) == 2
          and len(ts) == len(set(ts)) == 120 - 1 - gl.KEY_EVERY,
          repr((poss, len(_posts), len(ts))))

    # framing lost inside a record (its flag byte damaged below 0x80): the
    # records read from inside it are dropped, never decoded as points
    online(_tb)
    gl.LOG_DIR = _tb
    _tb_write(pr, rl[:120], per=60)
    raw = bytearray(_tb_raw(pr))
    gl._rd_open(0)
    offs = []
    with builtins.open(pr, "rb") as f:
        while gl._rd_rec(f):
            offs.append(gl._rd[0] + gl._rd[1])
    raw[offs[12]] = 0x30                 # a "0": a text line up to a 0x0A
    with builtins.open(pr, "wb") as f:
        f.write(raw)
    for _ in range(10):
        if os.path.exists(pr):
            pump()
    got = [(x["geometry"]["coordinates"][1], x["properties"]["timestamp"])
           for x in locations()]
    want = {}
    for ln in rl[:120]:
        f_ = ln.split(",")
        want["%s-%s-%sT%s:%s:%sZ" % (f_[5][0:4], f_[5][4:6], f_[5][6:8],
                                     f_[5][9:11], f_[5][11:13],
                                     f_[5][13:15])] = round(float(f_[0]), 6)
    wrong = [g for g in got if want.get(g[1]) != g[0]]
    check("TRKB: a record damaged below 0x80 drops the points up to the "
          "next keyframe, no wrong ones",
          not wrong and 60 <= len(got) < 120, repr((len(got), wrong[:2])))

    # a delta stream that keeps growing (damaged data): the state goes
    # invalid before a line outgrows the 96 bytes past the read buffer
    s_ = gl._ks
    ln = b"=1.000000,1.000000,1.0,1.0,1.0,20261101T000000Z\n"
    gl._rbuf[0:len(ln)] = ln
    gl._kb_rec(0, len(ln), s_)
    big = bytes([0x80, 0x80, 0x80, 0x80, 0x02])     # zigzag 2^29: +2^28
    rec = bytes([0xBE]) + big * 5
    gl._rbuf[0:len(rec)] = rec
    res = []
    try:
        for _ in range(8):
            r_ = gl._kb_rec(0, len(rec), s_)
            res.append((gl._kr[0], gl._kr[1]) if r_ > 0 else (0, 0))
    except Exception as e:
        res.append(repr(e))
    over = bytes([0x82, 0x80, 0x80, 0x80, 0x80, 0x04])  # 2^30: too big
    gl._rbuf[0:len(ln)] = ln
    gl._kb_rec(0, len(ln), s_)
    gl._rbuf[0:len(over)] = over
    r_over = (0, 0) if gl._kb_rec(0, len(over), s_) == 0 else "point"
    check("TRKB: deltas that keep growing (damaged data) make the state "
          "invalid before a line outgrows the decode buffer",
          all(isinstance(r, tuple) for r in res)
          and all(r[1] - gl._SCR <= 96 for r in res if r[1])
          and res[3] == (0, 0) and res[2][1] and res[4:] == [(0, 0)] * 4
          and r_over == (0, 0) and s_[0] is None,
          repr((res[:5], r_over)))

    # trk2gpx is the second decoder (the checks read .trb files through
    # it): the point lines it decodes from damaged bytes are the logger's
    # (_track_lines), and neither raises
    def _tb_pt(x):
        return x.endswith("\n") and x[:1] != "#" and 4 <= x.count(",") <= 5

    def _tb_fw(data):
        out = []
        gl._track_lines(io.BytesIO(data), "x.trb", 0, len(data), None,
                        lambda a, b: out.append(bytes(gl._rbuf[a:b]).decode(
                            "utf-8", "replace")))
        return [x for x in out if _tb_pt(x)]

    def _tb_py(data):
        try:
            return [x for x in trk2gpx.trb_lines(data) if _tb_pt(x)]
        except Exception as e:
            return repr(e)

    # an unknown text record (not "=", "-" or "#": a damaged one) makes the
    # state not valid, as _kb_rec: the deltas after it are no points
    ul = _tb_drive(40, 2, seed=11)
    ub = bytearray(gl.HEADERB.encode() + gl._kb_encode(ul, len(ul), None)
                   + gl.FOOTER.encode())
    gl._rd_open(0)
    uo = []
    with io.BytesIO(bytes(ub)) as f:
        while gl._rd_rec(f):
            uo.append((gl._rd[0] + gl._rd[1], gl._rd[0] + gl._rd[2]))
    a_, b_ = uo[6]                       # a delta read as a text line
    ub[a_:b_] = b"0" * (b_ - a_ - 1) + b"\n"
    up_, uf_ = _tb_py(bytes(ub)), _tb_fw(bytes(ub))
    check("TRKB: trk2gpx drops the deltas after an unknown text record "
          "(no points from a stale state), as the logger",
          up_ == uf_ == ul[:5], repr((len(up_), len(uf_),
                                      [x for x in up_ if x not in ul][:2])))

    # the logger's bounds on a delta: a varint over 5 bytes, a value over
    # 2 * _KB_MAX, a field past 9 digits, a time from FIX_S_END on; a
    # keyframe dated past it or with second 60 starts no deltas
    def _tb_case(kf, recs):
        return (gl.HEADERB + "=" + kf + "\n").encode() + bytes(recs) \
            + gl.FOOTER.encode()

    k0 = "1.000000,1.000000,1.0,1.0,1.0,20261101T000000Z"
    v29 = [0x80, 0x80, 0x80, 0x80, 0x02]           # 2^29 = 2 * _KB_MAX
    bnd = [_tb_case(k0, [0x82, 0x80, 0x80, 0x80, 0x80, 0x80, 0x01]),
           _tb_case(k0, [0x81] + [0xFF] * 9 + [0x01]),
           _tb_case(k0, [0x81] + v29 + [0x81] + v29 + [0x81] + v29
                    + [0x81] + v29),
           _tb_case(k0, [0x82, 0x81, 0x80, 0x80, 0x80, 0x02, 0x80]),
           _tb_case(k0, [0x82] + v29 + [0x82] + v29 + [0x82] + v29
                    + [0x82] + v29 + [0x80]),
           _tb_case("1.000000,1.000000,1.0,1.0,1.0,20871231T235959Z",
                    [0x80, 0x80]),
           _tb_case("1.000000,1.000000,1.0,1.0,1.0,20900101T000000Z",
                    [0x80, 0x80]),
           _tb_case("1.000000,1.000000,1.0,1.0,1.0,20261101T000060Z",
                    [0x80, 0x80]),
           _tb_case(k0, list(b"#" + b"x" * gl.LINE_MAX + b"\n") + [0x80])]
    bp = [_tb_py(x) for x in bnd]
    bf = [_tb_fw(x) for x in bnd]
    check("TRKB: trk2gpx applies the logger's bounds (varint length, value, "
          "9 digits, FIX_S_END, a keyframe's time, a line over LINE_MAX): "
          "the same points, no error",
          bp == bf and [len(x) for x in bf] == [1, 1, 4, 1, 4, 1, 1, 1, 1],
          repr([(len(a), len(b)) if a == b else (a, b)
                for a, b in zip(bp, bf)])[:600])

    # UPL-6: a keyframe whose time is no UTC time (month 13, hour 24:
    # _kb_parse rejects it, no deltas from it) is no point either, in the
    # upload and in trk2gpx (it went up as month 13)
    k6 = os.path.join(_tb, "20261101_150000.trb")
    k6_res = []
    for kf_ in ("47.1,19.1,1.0,1.0,1.0,20261301T000000Z",
                "47.1,19.1,1.0,1.0,1.0,20261101T240000Z",
                "47.1,19.1,1.0,1.0,1.0,20261101T235959Z"):
        with builtins.open(k6, "wb") as f:
            f.write(_tb_case(kf_, [0x80]))
        n6, ok6 = gl.read_batch(k6, 0)[:2]
        g6 = trk2gpx.convert(trk2gpx.trb_lines(_tb_raw(k6)), "x")
        k6_res.append((n6, ok6, g6.count("<trkpt")))
    os.remove(k6)
    check("UPL-6: a .trb keyframe dated out of range is no point (upload "
          "and trk2gpx), a good one and its delta are",
          k6_res == [(2, 0, 0), (2, 0, 0), (2, 2, 2)], repr(k6_res))

    # fuzz: single damaged bytes (a flipped bit, a random byte, a flag
    # byte's top bit cleared) in a TRK1 and a TRK2 drive, a fixed seed
    fr = _rnd.Random(7)
    fz = {}
    for fmt in (1, 2):
        fl = _tb_drive(100, fmt, seed=20 + fmt)
        fz[fmt] = (gl.HEADERB.encode() + gl._kb_encode(fl, len(fl), None)
                   + gl.FOOTER.encode())
    fbad = []
    for it in range(400):
        d_ = bytearray(fz[1 + it % 2])
        kind = fr.choice(("flip", "byte", "clear7"))
        p_ = fr.randrange(len(gl.HEADERB), len(d_) - len(gl.FOOTER))
        if kind == "flip":
            d_[p_] ^= 1 << fr.randrange(8)
        elif kind == "byte":
            d_[p_] = fr.randrange(256)
        else:
            while p_ < len(d_) - len(gl.FOOTER) - 1 and d_[p_] < 0x80:
                p_ += 1
            d_[p_] &= 0x7F
        try:
            fw_ = _tb_fw(bytes(d_))
        except Exception as e:
            fw_ = repr(e)
        py_ = _tb_py(bytes(d_))
        if fw_ != py_:
            fbad.append((it, kind, p_, fw_ if isinstance(fw_, str) else
                         len(fw_), py_ if isinstance(py_, str) else len(py_)))
    check("TRKB: fuzz - 400 single damaged bytes: trk2gpx decodes the very "
          "point lines of the logger, neither raises",
          not fbad, "%d differ: %r" % (len(fbad), fbad[:4]))

    # UPL-2: a flag byte followed by more bytes of 0x80 or more than a
    # record can hold (an erased flash block reads 0xFF) is a damaged
    # record of 1 byte. It ended where the read buffer did, so where it
    # ended depended on where the reading started: a .pos (offset,
    # keyframe) committed in it was no record start when read from its
    # keyframe ("pos misaligned", resent from 0, the same .pos again, for
    # ever: every later track blocked)
    def _tb_starts(data, at=0):
        out = []
        gl._rd_open(at)
        with io.BytesIO(data) as f:
            f.seek(at)
            while gl._rd_rec(f):
                out.append(gl._rd[0] + gl._rd[1])
        return out

    u2_base = fz[1]
    u2_d = [i_ for i_ in _tb_starts(u2_base) if u2_base[i_] >= 0x80]
    u2_bad = []
    for L_ in (31, 32, 40, 200, 600):
        for q_ in (u2_d[5], u2_d[40], u2_d[-3]):
            d_ = bytearray(u2_base)
            d_[q_ + 1:q_ + 1 + L_] = b"\xff" * L_
            d_ = bytes(d_)
            all_ = _tb_starts(d_)
            for s_ in all_[::3]:
                tail = _tb_starts(d_, s_)
                if tail != all_[all_.index(s_):]:
                    u2_bad.append((L_, q_, s_, "starts"))
                    break
            if _tb_fw(d_) != _tb_py(d_):
                u2_bad.append((L_, q_, "trk2gpx"))
    check("UPL-2: a run of bytes >= 0x80 after a flag byte: records start "
          "where they do whatever the read starts from, trk2gpx the same",
          not u2_bad, repr(u2_bad[:6]))

    # ... end to end: a track ending in an erased block goes up once
    online(_tb)
    gl.LOG_DIR = _tb
    _tb_write(pr, rl[:200], per=200)
    raw = _tb_raw(pr)
    q_ = [i_ for i_ in _tb_starts(raw) if raw[i_] >= 0x80][-5]
    with builtins.open(pr, "wb") as f:
        f.write(raw[:q_ + 1] + b"\xff" * 600)
    left_ = gl.up["left"] = gl.count_points()
    for _ in range(40):
        if os.path.exists(pr):
            pump()
    ts = [x["properties"]["timestamp"] for x in locations()]
    check("UPL-2: a .trb ending in a long run of 0xFF uploads once, its "
          "points all up, the file gone",
          not os.path.exists(pr) and len(ts) == len(set(ts))
          and len(ts) >= 190 and gl.up["left"] == 0,
          repr((os.path.exists(pr), len(ts), len(set(ts)), left_,
                gl.up["left"])))

    # out of memory while encoding is no write failure: the batch waits
    online(_tb)
    gl.LOG_DIR = _tb
    gl.log["file"] = os.path.join(_tb, "20261101_130000.trb")
    gl.log["fmt"] = 2
    gl.log["batch"] = list(rl[:10])
    enc_ = gl._kb_part                     # (flush encodes a chunk at a time)

    def _enc_oom(*a):
        raise MemoryError

    gl._kb_part = _enc_oom
    try:
        for _ in range(4):
            gl.flush()
    finally:
        gl._kb_part = enc_
    st_ = (gl.log["wfails"], gl.log["file"], len(gl.log["batch"]))
    gl.flush()
    check("TRKB: MemoryError while encoding keeps the batch and the file "
          "(no WFAIL_SWITCH); the next flush writes it",
          st_ == (0, os.path.join(_tb, "20261101_130000.trb"), 10)
          and gl.log["batch"] == [] and gl.log["points"] == 10, repr(st_))
    gl.log_reset()

    # TRKB-CHUNK: a flush encodes and writes its body KB_CHUNK bytes at a
    # time. One body of the whole batch needed a contiguous block that
    # grew with every MemoryError (the batch waited and grew) until the
    # MAX_BATCH cap dropped its oldest lines.
    online(_tb)
    gl.LOG_DIR = _tb
    pc_ = os.path.join(_tb, "20261101_140000.trb")
    cl_ = _tb_drive(240, 2, seed=11)
    gl.log["file"] = pc_
    gl.log["fmt"] = 2
    gl.log["batch"] = list(cl_)
    wmax_ = [0, 0]
    bo_ = builtins.open

    class _WSize(object):
        def __init__(self, f):
            self.f = f

        def seek(self, *a):
            return self.f.seek(*a)

        def tell(self):
            return self.f.tell()

        def read(self, *a):
            return self.f.read(*a)

        def write(self, data):
            wmax_[0] = max(wmax_[0], len(data))
            wmax_[1] += len(data)
            return self.f.write(data)

        def close(self):
            self.f.close()

    def _wsize_open(name, mode="r", *a, **k):
        f = bo_(name, mode, *a, **k)
        return _WSize(f) if name == pc_ and mode != "rb" else f

    builtins.open = _wsize_open
    try:
        gl.flush()
    finally:
        builtins.open = bo_
    dc_ = list(trk2gpx.trb_lines(_tb_raw(pc_)))
    check("TRKB-CHUNK: a flush of 240 lines writes at most ~KB_CHUNK bytes "
          "at a time, the same lines", wmax_[0] <= 256 + 64
          and wmax_[1] > 1000 and dc_[1:-1] == cl_ and not gl.log["batch"],
          repr((wmax_, len(dc_))))
    gl.log_reset()

    # the open file's name, and firmware from before TRK_BIN
    gl.st["date"], gl.st["time"] = "2026-11-01", "13:00:00"
    gl.start_file()
    nm1 = gl.log["file"]
    gl.TRK_BIN = False
    gl.start_file()
    nm2 = gl.log["file"]
    gl.TRK_BIN = True
    gl.log_reset()
    check("TRKB: new files are .trb (TRK_BIN; False: .trk as before), "
          "a suffix older firmware does not list",
          nm1.endswith("20261101_130000.trb") and nm2.endswith(".trk")
          and not nm1.endswith((".gpx", ".trk")), repr((nm1, nm2)))
finally:
    gl.LOG_DIR, gl.HIST_DIR, gl.UPLOAD_BATCH = _tb_saved
    shutil.rmtree(_tb, ignore_errors=True)
    gl.log_reset()
    gl.up_reset()


# --- The Heltec Wireless Tracker (UC6580) -----------------------------------
# The sentences here are the board's, measured 2026-11-04 (receiver UC6580I
# R6.0.0.0Build3700, indoors: no signal, so the fix sentences are made up).
def _heltec_logger(machine="Heltec Wireless Tracker with ESP32S3"):
    saved = getattr(sys.implementation, "_machine", None)
    sys.implementation._machine = machine
    try:
        spec = importlib.util.spec_from_file_location(
            "gps_logger_heltec", os.path.join(ROOT, "gps_logger.py"))
        m = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(m)
    finally:
        if saved is None:
            del sys.implementation._machine
        else:
            sys.implementation._machine = saved
    m.DIAG_FILE = os.path.join(diag_dir, "diag-h.log")
    m.DIAG_OLD = os.path.join(diag_dir, "diag-h.1")
    m.LOWIDF_MARK = os.path.join(diag_dir, "lowidf-h")
    m.HIST_DIR = os.path.join(diag_dir, "hist-h")
    m.HOME_APS_FILE = None
    m.WEB = False
    return m


class _UcUart(UART):
    """A UC6580: answers each command ($FAIL for those in bad, nothing while
    silent), PDTINFO with what the board's receiver says."""
    PDT = b"$PDTINFO,UC6580I,G1B1L1E1,V00,R6.0.0.0Build3700,2400615000268,20230729000954003*64\r\n"

    def __init__(self, bad=(), silent=False):
        self.bad, self.silent, self.cmds = bad, silent, []

    def write(self, data):
        data = bytes(data)
        self.cmds.append(data)
        if self.silent:
            return len(data)
        body = data[1:].split(b"*")[0].strip().decode()
        if body == "PDTINFO":
            _uart_rx.extend(self.PDT)
        if any(body.startswith(b) for b in self.bad):
            _uart_rx.extend(b"$FAIL,0*1E\r\n")
        else:
            _uart_rx.extend(b"$OK*04\r\n")
        return len(data)


def _uc_feed(m, *lines):
    for ln in lines:
        _uart_rx.extend(ln if isinstance(ln, bytes) else ln.encode())
        _uart_rx.extend(b"\r\n")
    m.read_uart()


def _nmea(body):
    c = 0
    for ch in body:
        c ^= ord(ch)
    return "$%s*%02X" % (body, c)


_pin_levels[:] = []
_uart_rx[:] = b""
hl = _heltec_logger()
check("HELTEC-BOARD: UART1 on 34/33 at 115200, Vext (GPIO3) high and the TFT "
      "backlight (GPIO21) off, before any UART use; no other board changes",
      hl.IS_HELTEC and hl.GPS_UC and not hl.IS_C6
      and (hl.GPS_UART, hl.GPS_TX, hl.GPS_RX, hl.BAUD) == (1, 34, 33, 115200)
      and [p[:2] for p in _pin_levels if p[0] in (3, 21)] == [(3, 1), (21, 0)]
      and not gl.GPS_UC and gl.BAUD == 9600,
      repr((hl.GPS_TX, hl.GPS_RX, hl.BAUD, _pin_levels[:4])))
check("HELTEC-PINS: only the receiver's and free pins for the GPS UART; a "
      "keep-alive pin not on the Vext, USB, LoRa or strapping pins",
      all(hl._gps_pin_ok(n, True) for n in (33, 34, 4, 48))
      and not any(hl._gps_pin_ok(n, True) for n in (0, 3, 8, 19, 20, 21, 26, 35, 38, 45))
      and hl.PB_PIN in hl._HELTEC_FREE and not hl._pin_taken(5)
      and all(hl._pin_taken(n) for n in (0, 3, 7, 18, 19, 21, 33, 34, 45)))
with open(os.path.join(ROOT, "main.py")) as f:
    _msrc = f.read()
check("HELTEC-SKIP: main.py's boot-bypass pin is GPIO7, free and not the "
      "download-mode button", "7 if \"Heltec\" in _M" in _msrc
      and 7 not in hl._HELTEC_FREE)

# the setup
hl.reset()
hl.uart = _UcUart()
_uart_rx[:] = b""
_ha_setup = hl.uart
hl.gps_setup()
_cmds = list(_ha_setup.cmds)
check("UC-SETUP: PDTINFO (no checksum: with one it gets no answer), then CFGNAV "
      "first (it resets the GSV rate), each with its checksum, no UBX byte; "
      "GST once a second, GSA and GSV at their pace, text notices off",
      _cmds[0] == b"$PDTINFO\r\n"
      and _cmds[1].startswith(b"$CFGNAV,200,200,1000*")
      and b"$CFGMSG,0,0,1*06\r\n" in _cmds
      and any(c.startswith(b"$CFGMSG,0,7,5*") for c in _cmds)
      and any(c.startswith(b"$CFGMSG,0,2,25*") for c in _cmds)
      and any(c.startswith(b"$CFGMSG,0,3,25*") for c in _cmds)
      and any(c.startswith(b"$CFGMSG,6,0,0*") for c in _cmds)
      and [c[1:].split(b"*")[0].decode() for c in _cmds[1:]] == [
          "CFGNAV,200,200,1000", "CFGMSG,0,0,1", "CFGMSG,0,4,1", "CFGMSG,0,1,0",
          "CFGMSG,0,5,0", "CFGMSG,0,2,25", "CFGMSG,0,3,25", "CFGMSG,0,7,5",
          "CFGMSG,6,0,0", "CFGMSG,6,1,0"]
      and not any(b"\xb5" in c for c in _cmds), repr(_cmds))
check("UC-SETUP: all answered: taken for now, NMEA mode, the receiver's model "
      "from PDTINFO",
      hl.st["setup_ok"] is True and hl.st["mode"] == "nmea"
      and hl.st["checked"] is False
      and hl.st["gps_ver"] == "UC6580I G1B1L1E1, R6.0.0.0Build3700",
      repr((hl.st["setup_ok"], hl.st["gps_ver"])))
_ha_setup.cmds[:] = []
hl.gps_setup()
check("UC-SETUP: the model is asked once", not any(
    c.startswith(b"$PDTINFO") for c in _ha_setup.cmds))

hl.reset()
hl.uart = _UcUart(silent=True)
_t0 = _clock[0]
hl.gps_setup()
check("UC-SILENT: no answer ends the setup at the first silent command after "
      "PDTINFO (the loop is not held up), and gps_check retries",
      hl.st["setup_ok"] is False and hl.st["checked"] is True
      and len(hl.uart.cmds) == 2 and _clock[0] - _t0 <= 3 * hl.UC_ACK_WAIT,
      repr((hl.st["setup_ok"], hl.uart.cmds, _clock[0] - _t0)))

hl.reset()
_hdl, _hdw = [], hl._diag_write
hl._diag_write = _hdl.append
hl.uart = _UcUart(bad=("CFGNAV",))
hl.gps_setup()
hl._diag_write = _hdw
_hd = "\n".join(_hdl)
check("UC-FAIL: a refused rate is said and is no \"not taken\" verdict; the "
      "rest of the setup goes on",
      hl.st["setup_ok"] is True and hl.st["rate_nak"] is True
      and "gps refused CFGNAV,200,200,1000" in _hd
      and any(c.startswith(b"$CFGMSG,0,7,5") for c in hl.uart.cmds), _hd[-200:])

# a restart: the banner the board prints after a power cycle
hl.reset()
hl.uart = _UcUart()
hl.gps_setup()
_b0, _e0 = hl.st["gps_boots"], hl.st["err"]
_uc_feed(hl, "", "UC6580I G1B1L1E1 COM1", "PN 2400615000268",
         "SN 20230729000954003", "HWVer 00", "FWVer R6.0.0.0Build3700",
         "Copyright (c), Unicore Communications Inc.", "All rights reserved.")
check("UC-RESTART: the plain text banner is one restart, not 7 bad lines; the "
      "setup is due again and asks the model anew, no UBX",
      hl.st["gps_boots"] == _b0 + 1 and hl.st["err"] == _e0
      and hl.st["extra"] >= 1 and hl.st["gps_ver"] is None
      and not any(b"\xb5" in c for c in hl.uart.cmds),
      repr((hl.st["gps_boots"], hl.st["err"], hl.st["extra"])))

# 5 Hz with a fix: the verdict the u-blox setup gets
hl.reset()
hl.uart = _UcUart()
hl.gps_setup()
_ok0 = hl.st["ok"]
for _s in range(5):
    for _f in range(5):
        _t = "1200%02d.%02d" % (_s, _f * 20)
        _uc_feed(hl,
                 _nmea("GNRMC,%s,A,4711.1214,N,01825.6160,E,0.05,,041126,,,A,V" % _t),
                 _nmea("GNGGA,%s,4711.1214,N,01825.6160,E,1,14,0.80,120.5,M,40.0,M,," % _t))
        if _f == 0:
            _uc_feed(hl, _nmea("GNGST,%s,0.8,,,,1.4,1.6,2.5" % _t))
        _clock[0] += 200
_clock[0] += hl.GPS_VERIFY
hl._setup_check(_clock[0])
check("UC-5HZ: GGA, RMC and GST at 5 Hz pass the setup's check (5 fixes a "
      "second, GST seen), nothing counted as not taken",
      hl.st["checked"] is True and hl.st["extra"] == 0 and hl.st["hz5"] > 0
      and hl.st["err"] == 0 and hl.st["std"] is not None and hl.st["ok"] > _ok0,
      repr((hl.st["checked"], hl.st["extra"], hl.st["hz1"], hl.st["hz5"], hl.st["err"])))

# no fix yet: its RMC carries a date that is not today
hl.reset()
hl.uart = _UcUart()
_uc_feed(hl, _nmea("GNRMC,090328.00,V,,,,,,,170126,,,N,V"),
         _nmea("GNGGA,090328.00,,,,,0,00,99.99,,,,,,"),
         _nmea("GNGST,090328.00,0.0,,,,3750000,3750000,3750000"))
_clock[0] += 1100
_uc_feed(hl, _nmea("GNRMC,090329.00,V,,,,,,,170126,,,N,V"),
         _nmea("GNGGA,090329.00,,,,,0,00,99.99,,,,,,"))
_clock[0] += 1100
_uc_feed(hl, _nmea("GNRMC,090330.00,V,,,,,,,170126,,,N,V"))
check("UC-NOFIX: the receiver's made-up date before a fix (17 Jan 2026, no "
      "position) is not a date or a fix", hl.st["pos_at"] is None
      and hl.st["date"] is None and hl.st["err"] == 0,
      repr((hl.st["pos_at"], hl.st["date"], hl.st["err"])))

# the satellite list from GSV: two bands of one satellite, then GSA's used ones
hl.reset()
hl.uart = _UcUart()
hl.gps_setup()
_uc_feed(hl, _nmea("GNGSA,A,3,14,17,,,,,,,,,,,1.2,0.8,0.9,1"),
         _nmea("GNGSA,A,3,25,,,,,,,,,,,,1.2,0.8,0.9,4"),
         _nmea("GPGSV,1,1,03,14,77,011,40,17,56,319,35,30,36,201,,1"),
         _nmea("GPGSV,1,1,03,14,77,011,44,17,56,319,,30,36,201,,8"),
         _nmea("GBGSV,1,1,01,25,75,249,38,1"),
         _nmea("GAGSV,1,1,01,13,42,203,,7"))
_e1 = hl.st["extra"]
_clock[0] += 200                         # no pause: GSV at 1 Hz with a fix
_uc_feed(hl, _nmea("GPGSV,1,1,00,1"))    # the next round closes this one
_svs = {(v[6], v[0]): v for v in (hl.st["svs"] or [])}
_uok = hl.st["ok"]
_uc_feed(hl, _nmea("GPGSV,1,1,01,14,77,011,99,1"))
_clock[0] += hl.GSV_EVERY * 1000 + 1
_skip = hl.st["ok"] == _uok + 1 and not hl._uc_sv["new"]
_uc_feed(hl, _nmea("GAGSV,1,1,01,13,42,203,31,7"))      # the end of a round
_mid = not hl._uc_sv["new"]
_uc_feed(hl, _nmea("GPGSV,1,1,01,14,77,011,50,1"))
_uc_feed(hl, _nmea("GPGSV,1,1,01,14,77,011,50,1"))
_again = [v[1] for v in hl.st["svs"]] == [50]
check("UC-GSV: one list from a round: a satellite once with its strongest band, "
      "used from GSA, NAV-SAT's shape and gnssIds; GSV is no sentence the "
      "setup turned off",
      _svs.get((0, 14)) == (14, 44, 77, 11, True, False, 0)
      and _svs.get((0, 17)) == (17, 35, 56, 319, True, False, 0)
      and _svs.get((0, 30)) == (30, 0, 36, 201, False, False, 0)
      and _svs.get((3, 25)) == (25, 38, 75, 249, True, False, 3)
      and _svs.get((2, 13)) == (13, 0, 42, 203, False, False, 2)
      and _e1 == 0 and hl.st["extra"] == 0, repr((_svs, _e1, hl.st["extra"])))
check("UC-GSV-SKIP: GSV_EVERY s of GSV not decoded after a list (1 Hz with a "
      "fix); then not from a round's middle, a whole round again",
      _skip and _mid and _again, repr((_skip, _mid, _again, hl.st["svs"])))
hl.uart.cmds[:] = []
hl.sv_poll(_clock[0])
check("UC-NO-UBX: the satellite poll writes nothing (no UBX to a UC6580)",
      hl.uart.cmds == [] and hl.st["sv_miss"] == 0, repr(hl.uart.cmds))

# GPS_SYS: a mask set once, only when the receiver has another
class _UcSys(_UcUart):
    has = "H35155"

    def write(self, data):
        n = super().write(data)
        if bytes(data) == b"$CFGSYS\r\n":
            del _uart_rx[-8:]
            _uart_rx.extend(_nmea("CFGSYS," + self.has).encode() + b"\r\n$OK*04\r\n")
        return n


hl.reset()
hl.UC_SYS = None
hl._gps_sys_env({"GPS_SYS": "h135155"})
_w = hl.UC_SYS
hl._gps_sys_env({"GPS_SYS": "xyz"})
_bad = hl.UC_SYS
hl.uart = _UcSys()
hl.gps_setup()
_c1 = list(hl.uart.cmds)
hl.gps_setup()
_c2 = hl.uart.cmds[len(_c1):]
hl.reset()
hl.uart = _UcSys()
hl.uart.has = "H135155"
hl.gps_setup()
_c3 = list(hl.uart.cmds)
hl.UC_SYS = None
hl.reset()
hl.uart = _UcUart()
hl.gps_setup()
_c4 = list(hl.uart.cmds)
hl.UC_SYS = 0x135155
hl.reset()
hl.uart = _UcSys(bad=("CFGSYS,",))       # the board's: $FAIL for the SBAS bit
_rl, _rw = [], hl._diag_write
hl._diag_write = _rl.append
hl.gps_setup()
hl._diag_write = _rw
_c5 = list(hl.uart.cmds)
_r5 = hl.st["setup_ok"]
hl.UC_SYS = None
check("UC-SYS-REFUSED: a refused mask is said as refused and the setup goes on: "
      "the receiver is not left at its defaults (1 Hz, no GST)",
      any(l.startswith("gps refused CFGSYS,") for l in _rl)
      and not any(l.startswith("gps systems") for l in _rl)
      and any(c.startswith(b"$CFGNAV") for c in _c5) and _r5 is True, repr((_rl, _c5[:4])))
_set = lambda cs: [c for c in cs if c.startswith(b"$CFGSYS,")]
check("UC-SYS: GPS_SYS sets the mask once a boot and only when it differs (it "
      "restarts the receiver, the setup then comes back); without it none is sent",
      _w == 0x135155 and _bad == 0x135155 and len(_set(_c1)) == 1
      and _set(_c1)[0].startswith(b"$CFGSYS,h135155*") and not _set(_c2)
      and any(c.startswith(b"$CFGNAV") for c in _c2) and not _set(_c3)
      and any(c.startswith(b"$CFGNAV") for c in _c3)
      and not any(c.startswith(b"$CFGSYS") for c in _c4), repr((_c1, _c3[:3])))

# the first minute of a fix counts only moving (its GST says 0.3 m while 4 m off)
def _uc_fixes(m, n, kn="0.05", q=1):
    """n seconds of a fix through the real parser."""
    for _i in range(n):
        _c = _clock[0] // 1000 % 86400
        _t = "%02d%02d%02d.00" % (_c // 3600, _c // 60 % 60, _c % 60)
        _uc_feed(m, _nmea("GNRMC,%s,A,4711.1214,N,01825.6160,E,%s,,041126,,,A,V" % (_t, kn)),
                 _nmea("GNGGA,%s,4711.1214,N,01825.6160,E,%d,14,0.80,120.5,M,40.0,M,," % (_t, q)))
        _clock[0] += 1000


hl.reset()
hl.uart = _UcUart()
_clock[0] += 1000
_uc_feed(hl, _nmea("GNRMC,090328.00,V,,,,,,,170126,,,N,V"))
_clock[0] += 1000
_uc_feed(hl, _nmea("GNRMC,090329.00,V,,,,,,,170126,,,N,V"))
_v0 = hl.st["fix_first"]
_clock[0] = (_clock[0] // 1000 + 1) * 1000
_uc_fixes(hl, 3)
_f0 = hl.st["fix_first"]
_u0 = hl._usable(_clock[0])
_uc_fixes(hl, 2, kn="27.0")              # 50 km/h: a car's trip keeps its start
_u1 = hl._usable(_clock[0])
_uc_fixes(hl, 2)
_f1 = hl.st["fix_first"]
_u2 = hl._usable(_clock[0])
_clock[0] += hl.UC_SETTLE
_uc_fixes(hl, 2)
_u3 = hl._usable(_clock[0])
_s3 = hl.st["settled"]
check("UC-SETTLE: no fix does not start it; the first fix does, later ones do not "
      "move it; slow points wait UC_SETTLE, above UC_SETTLE_KMH they count at once, "
      "after it all do",
      _v0 is None and _f0 is not None and _f1 == _f0 and _u0 is False
      and _u1 is True and _u2 is False and _u3 is True and _s3 is True,
      repr((_v0, _f0, _f1, _u0, _u1, _u2, _u3, _s3)))
# a day later: no second window (it was in the 24 h stamp clean-up)
_clock[0] += hl.TICKS_CAP + 1000
hl._stamps_old(_clock[0])
_uc_fixes(hl, 2)
_u4 = hl._usable(_clock[0])
_nofirst = hl.st["fix_first"] == _f0
gl.reset()
gl.st.update(speed=2.0)
_u5 = gl._usable(_clock[0])
check("UC-SETTLE-DAY: the rule does not come back after TICKS_CAP of uptime; other "
      "receivers have none", _u4 is True and _nofirst and _u5 is True,
      repr((_u4, hl.st["fix_first"], _f0, _u5)))
# a restart in the middle of a second: its fixes do not start the rule
hl.st.update(settled=True, fix_first=5)
_uc_fixes(hl, 1)
_uc_feed(hl, "FWVer R6.0.0.0Build3700")
_r0 = (hl.st["fix_first"], hl.st["settled"])
_clock[0] += 1000
_uc_feed(hl, _nmea("GNRMC,090330.00,V,,,,,,,170126,,,N,V"))
_clock[0] += 1000
_uc_feed(hl, _nmea("GNRMC,090331.00,V,,,,,,,170126,,,N,V"))
_r1 = hl.st["fix_first"]
check("UC-SETTLE-RESTART: the banner starts the rule anew, and the unfinished second "
      "from before it does not count as the first fix",
      _r0 == (None, False) and _r1 is None, repr((_r0, _r1)))

# a restart whose banner was lost: no GST with sentences coming
hl.reset()
hl.uart = _UcUart()
hl.gps_setup()
_dl2, _dw2 = [], hl._diag_write
hl._diag_write = _dl2.append
_clock[0] += hl.UC_GST_LOST - 1000
hl.st["rx"] = _clock[0]
hl._uc_watch(_clock[0])
_w0 = hl.st["extra"]
_clock[0] += 2000
hl.st["rx"] = None                       # silent: another case
hl._uc_watch(_clock[0])
_w1 = hl.st["extra"]
hl.st["rx"] = _clock[0]
hl.st["gst_at"] = _clock[0] - 1000       # GST comes: all is well
hl._uc_watch(_clock[0])
_w2 = hl.st["extra"]
hl.st["gst_at"] = _clock[0] - hl.UC_GST_LOST
hl._uc_watch(_clock[0])
_w3 = hl.st["extra"]
hl._uc_watch(_clock[0])
_cn = len(hl.uart.cmds)
hl.st["setup_at"] = _clock[0] - hl.st["setup_wait"]
hl.gps_check(_clock[0])
_w4 = len(hl.uart.cmds) > _cn and hl.st["extra"] == 0
hl._diag_write = _dw2
check("UC-WATCH: no GST for UC_GST_LOST with sentences coming: gps_check sets it up "
      "again, said; not before, not when silent, not while GST comes",
      _w0 == 0 and _w1 == 0 and _w2 == 0 and _w3 == 1 and _w4
      and _dl2 == ["gps setup lost: no GST"], repr((_w0, _w1, _w2, _w3, _w4, _dl2)))
hl.reset()
_wait0 = _clock[0]
hl.st.update(svs_at=None)
hl.uart = _UcUart()
hl._sv_fresh()
check("UC-NO-WAIT: a page view does not hold the loop for a list nothing can ask for",
      _clock[0] == _wait0, repr(_clock[0] - _wait0))
_ipolls = []


class _Ipoll(object):
    def ipoll(self, t):
        _ipolls.append(t)
        return iter(())


hl._usb_in = _Ipoll()
hl.gps_check(_clock[0])
_ip_ok = _ipolls == [0]


class _IpollBad(object):
    def ipoll(self, t):
        raise MemoryError()


hl._usb_in = _IpollBad()
hl._usb_drain()
hl._usb_in = None
check("USB-DRAIN: every gps_check reads stdin on the Heltec, without a list per pass "
      "(ipoll); no MemoryError leaves it", _ip_ok, repr(_ipolls))

# GSA: a burst is one round, wherever the second's clock falls in it
hl.reset()
hl.uart = _UcUart()
_clock[0] += 1000
_uc_feed(hl, _nmea("GNGSA,A,3,14,17,,,,,,,,,,,1.2,0.8,0.9,1"))
hl.st["gsa_at"] = _clock[0] - 1000       # the next one a second after the first
_uc_feed(hl, _nmea("GNGSA,A,3,25,,,,,,,,,,,,1.2,0.8,0.9,4"))
_u1 = sorted(hl._uc_sv["used"])
_clock[0] += 200
_uc_feed(hl, _nmea("GNGSA,A,3,30,,,,,,,,,,,,1.2,0.8,0.9,1"))
check("UC-GSA: the satellites in use of one burst stay together past a second's "
      "boundary; a pause starts the next round's",
      _u1 == sorted([14, 17, 3 << 8 | 25]) and hl._uc_sv["used"] == [30],
      repr((_u1, hl._uc_sv["used"])))

# the display: the fonts, the values, what goes to the panel
import gps_tft as _ft
import gps_tft_font as _ff
_fo = []
for _f in (_ff.HERO, _ff.NUM, _ff.TXT, _ff.LAB, _ff.WORD):
    _chars, _wd, _of, _dat, _h, _base = _f
    _end = 0
    _ok1 = len(_wd) == len(_chars) and len(_of) == 2 * len(_chars) and 0 < _base <= _h
    for _i in range(len(_chars)):
        _o = _of[_i * 2] | _of[_i * 2 + 1] << 8
        _ok1 = _ok1 and _o == _end
        _end = _o + ((_wd[_i] + 1) // 2) * _h
    _fo.append(_ok1 and _end == len(_dat))
_shows = "0123456789-:kész KM/Hműhold"
check("TFT-FONT: every glyph's offset and size add up, 16 levels; every character the "
      "screen shows is in its font",
      all(_fo) and all(c in _ff.HERO[0] and c in _ff.NUM[0] for c in "0123456789-")
      and all(c in _ff.TXT[0] for c in "0123456789:-kész ")
      and all(c in _ff.LAB[0] for c in "KM/HGPSműhold")
      and all(c in _ff.WORD[0] for w, _t in _ft.QUALITY for c in w)
      and len(set(_ff.HERO[3])) > 2, repr(_fo))

hl.reset()
hl.log_reset()
hl.wifi_reset()
hl.up_reset()
hl.LOG_DIR = os.path.join(diag_dir, "gpx-h")
os.makedirs(hl.LOG_DIR, exist_ok=True)
hl.up["url"] = "https://x"
hl.up["left"] = 1284
hl.wifi["state"] = "off"
hl.st.update(date="2026-11-04", time="10:42:10", valid=True, lat=47.1, lon=18.4,
             pos_at=_clock[0], speed=46.6, hdop=0.968, used=22, view={"GP": 28},
             alt=182.4, rx=_clock[0], fixtype=3)
_s1 = hl._tft_state(_clock[0])
_s2 = hl._tft_state(_clock[0] + hl.STALE + 1)
hl.wifi.update(state="connected", ssid="krumpli", rssi=-55, ip="192.168.1.60")
hl.up["left"] = 0
_s3 = hl._tft_state(_clock[0] + hl.STALE + 1)
_lv = []
for _r in (-55, -60, -61, -70, -71, -80, -81, None):
    _lv.append(hl._wifi_level(_r, None))
_hy = [hl._wifi_level(-61, 3), hl._wifi_level(-60, 2), hl._wifi_level(-58, 2), hl._wifi_level(-57, 2),
       hl._wifi_level(-85, 3), hl._wifi_level(-40, 0), hl._wifi_level(-67, 1), hl._wifi_level(-77, 0)]
hl._tft_b[2] = None
hl.up["url"] = None
_s4 = hl._tft_state(_clock[0])
hl.up["url"] = "https://x"
hl.up["left"] = 12345
_s5 = hl._tft_state(_clock[0])
hl.up["left"] = 0
_gq = []
for _hd, _us in ((0.9, 26), (1.5, 10), (1.5, 9), (1.6, 26), (3.0, 12), (3.1, 26), (9.0, 8), (None, 26)):
    hl.st.update(hdop=_hd, used=_us, svs=None)
    _gq.append(hl._gps_quality(True))
_gq0 = hl._gps_quality(False)
check("TFT-STATE: speed rounded with a fix, -- without; the clock; WiFi off or its "
      "level by the signal, down at once, up only past a margin; what waits, short when long, done or nothing set up; the "
      "GPS's quality from HDOP and the satellites in the fix",
      _s1[:4] == ("47", True, 22, "11:42") and _s1[4] is None and _s1[6:8] == ("1284", True)
      and _s2[0] == "--" and _s2[1] is False
      and _s3[4] == 3 and _s3[6:8] == ("kész", False)
      and _lv == [3, 3, 2, 2, 1, 1, 0, 0] and _hy == [2, 2, 2, 3, 0, 3, 2, 1]
      and _s4[6] == " " and _s5[6] == "12k"
      and _gq == [3, 3, 2, 2, 2, 1, 1, 1] and _gq0 == 0 and _s1[5] == 3 and _s2[5] == 0,
      repr((_s1, _s2, _s3, _lv, _s4, _s5, _gq, _gq0)))
_ls = lambda d, t: hl.st.update(date=d, time=t) or hl._local_hm()
check("TFT-CLOCK: Hungarian time from the fix: CET in winter, CEST in summer, "
      "the switch on the last Sunday at 01:00 UTC, past midnight, no date",
      _ls("2026-11-04", "10:42:10") == "11:42" and _ls("2026-07-01", "23:10:00") == "01:10"
      and _ls("2026-03-29", "00:59:00") == "01:59" and _ls("2026-03-29", "01:00:00") == "03:00"
      and _ls("2026-10-25", "00:59:00") == "02:59" and _ls("2026-10-25", "01:00:00") == "02:00"
      and _ls(None, None) == "--:--")
hl.st["rx"] = _clock[0] - hl.NO_DATA
check("TFT-CLOCK-SILENT: a silent receiver's clock shows --:--, not a stopped time",
      _ls("2026-11-04", "10:42:10") == "--:--")
hl.st["rx"] = _clock[0]

# the flash: per cent taken, how long a drive still fits
_sv0, _hn0, _fsz0 = os.statvfs, hl._hist_names, hl._file_size
_sv_n = []


def _store(free_kb, hist=(), total_kb=4544, at=None):
    os.statvfs = lambda p: _sv_n.append(p) or (1024, 1024, total_kb, free_kb, free_kb, 0, 0, 0, 0, 255)
    hl._hist_names = lambda: [n for n, _z in hist]
    hl._file_size = lambda p: dict(hist).get(p.rsplit("/", 1)[-1])
    hl._tft_fs[:] = [None, None]
    return hl._store_state(_clock[0] if at is None else at)


_stores = (_store(4412), _store(1136), _store(182), _store(64), _store(33), _store(32), _store(0))
# an older day's file counts as room, today's and a vanished one do not
_sh = _store(100, (("20261101.trb", 1024 * 1024), ("20261102.trb", None), ("20261103.trb", 500 * 1024)))
_sh0 = _store(100, (("20261103.trb", 1024 * 1024),))
_sv_n[:] = []
_sa = _store(4412, at=_clock[0])
_sb = hl._store_state(_clock[0] + hl.TFT_STORE - 1)
_n1 = len(_sv_n)
_sc = hl._store_state(_clock[0] + hl.TFT_STORE)
_n2 = len(_sv_n)


def _sv_fail(p):
    raise OSError(5)


os.statvfs = _sv_fail
hl._tft_fs[:] = [None, None]
_sf = hl._store_state(_clock[0])
_st9 = hl._tft_state(_clock[0])
os.statvfs, hl._hist_names, hl._file_size = _sv0, _hn0, _fsz0
hl._tft_fs[:] = [None, None]
check("TFT-STORE: the flash in per cent and the time a 1 Hz drive still fits: days from "
      "48 h, then hours, under an hour, full at the reserve; older history days count as "
      "room, today's not; statvfs once per TFT_STORE; unreadable: nothing, no exception",
      _stores == ((3, "7 nap"), (75, "44 óra"), (96, "6 óra"), (99, "1 óra"), (99, "<1 óra"),
                  (100, "tele"), (100, "tele"))
      and _sh == (76, "44 óra") and _sh0 == (98, "2 óra")
      and _sa == _sb == _sc and _n1 == 1 and _n2 == 2
      and _sf is None and _st9[8] is None
      and all(c in _ff.TXT[0] for _p, _l in _stores for c in "%d%%" % _p + _l),
      repr((_stores, _sh, _sh0, _n1, _n2, _sf, _st9[8])))

# the drawing, on a model of the panel: real pixels, at real coordinates
_order = []
_pin0 = _ft.Pin


class _BlPin(Pin):
    def __init__(self, n, *a, **k):
        super().__init__(n, *a, **k)
        if n == 21:
            _order.append((k.get("value"), len(_TftSpi.out)))


_ft.Pin = _BlPin
_TftSpi.out[:] = []
_TftSpi.panel.clear()
_TftSpi.fail = False
_ft.start()
_cmds0 = list(_TftSpi.out)
_ft.Pin = _pin0
_bg_all = len(_TftSpi.panel) == 160 * 80 and set(_TftSpi.panel.values()) == {_ft.BG}
_ft.set_state("47", True, 22, "11:42", None, 3, "1284", True)
_TftSpi.frames = 0
_ft.frame()
_f1 = _TftSpi.frames
_px = lambda x, y: _TftSpi.panel.get((x + _ft.XO, y + _ft.YO))
_cnt = lambda x0, y0, x1, y1, c: sum(1 for y in range(y0, y1) for x in range(x0, x1) if _px(x, y) == c)
_org = _ft._gs4_palette("org").pixel(15, 0)
_txt = _ft._gs4_palette("txt").pixel(15, 0)
_soft = _ft._gs4_palette("soft").pixel(15, 0)
_grn = _ft._gs4_palette("grn").pixel(15, 0)
_mid = _ft._gs4_palette("org").pixel(7, 0)
_speed = _cnt(4, 0, 60, 41, _org)
_aa = _cnt(4, 0, 60, 41, _mid)
_num = _cnt(130, 2, 160, 18, _txt)
_clockp = _cnt(125, 28, 160, 40, _soft)
_iconp = _cnt(_ft.WIFI_X, _ft.WIFI_Y, _ft.WIFI_X + 12, _ft.WIFI_Y + 12, _ft.DIM)
_barc = _c0 = _ft._c(_ft._TONES["grn"])
_bh = [sum(1 for y in range(46, 76) if _px(5 + i * 8 + 2, y) == _barc) for i in range(4)]
_word = _cnt(42, 62, 100, 76, _grn)
_gpsl = _cnt(42, 50, 70, 63, _ft._gs4_palette("mut").pixel(15, 0))
_upp = _cnt(126, 62, 160, 76, _org)
_upi = _cnt(100, 62, 158, 76, _ft._c(_ft._TONES["org"]))
_TftSpi.frames = 0
_ft.frame()
_f2 = _TftSpi.frames
_ft.set_state("48", True, 22, "11:42", None, 3, "1284", True)
_TftSpi.frames = 0
_ft.frame()
_f3 = _TftSpi.frames
_ft.set_state("48", True, 22, "11:42", None, 1, "1284", True)
_TftSpi.frames = 0
_ft.frame()
_f4 = _TftSpi.frames
_weak = [sum(1 for y in range(46, 76) if _px(5 + i * 8 + 2, y) == _ft._c(_ft._TONES["org"])) for i in range(4)]
_ft.set_state("--", False, 0, "--:--", None, 0, "kész", False)
_ft.frame()
_nofix = (_cnt(4, 0, 60, 41, _org), _cnt(130, 2, 160, 18, _ft._gs4_palette("red").pixel(15, 0)) > 5,
          [sum(1 for y in range(46, 76) if _px(5 + i * 8 + 2, y) == _ft.OFF) for i in range(4)],
          _cnt(42, 62, 100, 76, _ft._gs4_palette("red").pixel(15, 0)) > 5,
          _cnt(126, 62, 160, 76, _ft._gs4_palette("soft").pixel(15, 0)) > 3)
check("TFT-INIT: 16 bits a pixel and landscape, the whole panel cleared, the backlight "
      "only after that (no flash of random RAM)",
      b"\x36" in _cmds0 and b"\x68" in _cmds0 and b"\x05" in _cmds0 and _bg_all
      and _order[0][0] == 0 and _order[-1][0] == 1 and _order[-1][1] == len(_cmds0),
      repr((_order, len(_cmds0), _bg_all)))
check("TFT-DRAW: the measured offset; the speed orange and anti-aliased, the satellites, "
      "the clock and the dim icon where they belong; strong GPS: four rising green bars, "
      "the word and the label; what waits in orange with its arrow",
      (_ft.XO, _ft.YO) == (1, 26) and _speed > 200 and _aa > 0 and _num > 15
      and _clockp > 5 and _iconp > 10 and _bh == [6, 12, 18, 24] and _word > 5
      and _gpsl > 5 and _upp > 5 and _upi > 10,
      repr((_speed, _aa, _num, _clockp, _iconp, _bh, _word, _gpsl, _upp, _upi)))
check("TFT-ZONES: a frame draws both zones (3 + 3 bands), an unchanged one none; a new "
      "speed only the top, a new quality only the lower zone, weak lights two bars",
      _f1 == 6 and _f2 == 0 and _f3 == 3 and _f4 == 3 and _weak == [6, 12, 0, 0],
      repr((_f1, _f2, _f3, _f4, _weak)))
check("TFT-NOFIX: no fix: no orange, the satellite count red, all bars off, the word "
      "red, done in grey", _nofix == (0, True, [6, 12, 18, 24], True, True), repr(_nofix))

# the flash's gauge: its length by the per cent, its colour by the level
_gx = lambda c: sum(1 for x in range(_ft.STORE_X, _ft.STORE_X + _ft.STORE_W) if _px(x, 61) == c)
_gr, _gy, _gd = (_ft._c(_ft._TONES[t]) for t in ("grn", "yel", "red"))
_tp = lambda t: _ft._gs4_palette(t).pixel(15, 0)
_gs = []
for _p, _l in ((3, "7 nap"), (50, "3 nap"), (70, "44 óra"), (89, "9 óra"), (90, "8 óra"), (100, "tele")):
    _ft.set_state("47", True, 22, "11:42", 3, 3, "kész", False, (_p, _l))
    _ft.frame()
    _gs.append((_gx(_gr), _gx(_gy), _gx(_gd), _gx(_ft.OFF), _cnt(_ft.STORE_X, 48, 159, 58, _tp("soft")) > 5,
                _cnt(_ft.STORE_X, 48, 159, 58, _tp("yel")) > 5, _cnt(_ft.STORE_X, 48, 159, 58, _tp("red")) > 5))
_TftSpi.frames = 0
_ft.frame()
_gs0 = _TftSpi.frames
_ft.set_state("47", True, 22, "11:42", 3, 3, "kész", False, (100, "tele"))
_ft.frame()
_up_gone = _cnt(_ft.STORE_X, 66, 159, 76, _tp("soft")) > 3
_ft.set_state("47", True, 22, "11:42", 3, 3, "kész", False, (99, "<1 óra"))
_ft.frame()
_w = _ft.width(_ft._TXT, "99%") + _ft.width(_ft._TXT, "<1 óra")
_ft.set_state("47", True, 22, "11:42", 3, 3, "kész", False, None)
_ft.frame()
_none = (_gx(_gr), _gx(_gy), _gx(_gd), _gx(_ft.OFF), _cnt(_ft.STORE_X, 48, 159, 58, _tp("soft")))
check("TFT-STORE-DRAW: the gauge's length by the per cent, green, yellow from 70, red from "
      "90, the text with it, the upload line left alone, an unchanged state sends nothing, "
      "the longest two texts fit; unreadable: no gauge, no text",
      _gs == [(1, 0, 0, 55, True, False, False), (28, 0, 0, 28, True, False, False),
              (0, 39, 0, 17, False, True, False), (0, 49, 0, 7, False, True, False),
              (0, 0, 50, 6, False, False, True), (0, 0, 56, 0, False, False, True)]
      and _gs0 == 0 and _up_gone and _w < _ft.STORE_W and _none == (0, 0, 0, 0, 0),
      repr((_gs, _gs0, _up_gone, _w, _none)))
_ft.set_state("47", True, 22, "11:42", None, 3, "kész", False)
_ft.frame()

# the WiFi icon: still, its arcs by the level; a new level sends only the icon
_arcs = lambda c: _cnt(_ft.WIFI_X, _ft.WIFI_Y, _ft.WIFI_X + 12, _ft.WIFI_Y + 12, c)
_ft.set_state("47", True, 22, "11:42", 3, 3, "kész", False)
_ft.frame()
_a3 = _arcs(_ft.ORG)
_TftSpi.out[:] = []
for _i in range(20):
    _ft.frame()
_still3 = sum(1 for b in _TftSpi.out if len(b) > 8) == 0
_ft.set_state("47", True, 22, "11:42", 1, 3, "kész", False)
_TftSpi.out[:] = []
_ft.frame()
_sent1 = [len(b) for b in _TftSpi.out if len(b) > 8]
_a1 = _arcs(_ft.ORG)
_ft.set_state("47", True, 22, "11:42", None, 3, "kész", False)
_ft.frame()
_off = (_arcs(_ft.ORG), _arcs(_ft.DIM) > 10)
check("TFT-WIFI: the icon stands: 20 frames send nothing; a lower level sends only the "
      "12x12 icon with fewer arcs lit; no WiFi: all dim",
      _still3 and _sent1 == [288] and _a3 > _a1 > 0 and _off == (0, True),
      repr((_still3, _sent1, _a3, _a1, _off)))

# a panel gone wrong: dark, and each action counted when it runs
hl._tft = _ft
hl._tft_t[:] = [None, None, 0, 0]
_TftSpi.fail = True
_dl, _dw = [], hl._diag_write
hl._diag_write = _dl.append
_ft._D.top = None
_pin_levels[:] = []
for _i in range(hl.TFT_MAX_ERR):
    _clock[0] += hl.TFT_FRAME
    hl.tft_tick(_clock[0])               # the frame fails
    if _i < hl.TFT_MAX_ERR - 1:
        hl.tft_tick(_clock[0] + 1)       # nothing due: must not clear the count
hl._diag_write = _dw
_TftSpi.fail = False
_dark = _pin_levels and [p[:2] for p in _pin_levels if p[0] == 21][-1:] == [(21, 0)]
check("TFT-FAIL: TFT_MAX_ERR failed frames in a row, with idle ticks between, turn the "
      "display dark for the boot (no frozen speed lit), said once, nothing leaves tft_tick",
      hl._tft is None and len(_dl) == 1 and _dl[0].startswith("bug tft ") and _dark,
      repr((hl._tft, _dl, _dark)))
_st0 = hl._tft_state
hl._tft = _ft
hl._tft_t[:] = [None, None, 0, 0]
hl._tft_state = lambda now: (_ for _ in ()).throw(ValueError("state"))
hl._diag_write = lambda l: None
_pin_levels[:] = []
for _i in range(hl.TFT_MAX_ERR):
    _clock[0] += hl.TFT_STATE
    hl.tft_tick(_clock[0])               # a good frame between must not clear it
hl._tft_state, hl._diag_write = _st0, _dw
check("TFT-FAIL-STATE: values failing keep counting though frames succeed: no old speed "
      "redrawn for ever", hl._tft is None
      and [p[:2] for p in _pin_levels if p[0] == 21][-1:] == [(21, 0)], repr(hl._tft))
_ft_seen = []
gl._tft = None
sys.modules["gps_tft"] = types.SimpleNamespace(start=lambda: _ft_seen.append("start"))
gl._tft_start()
sys.modules["gps_tft"] = _ft
with open(os.path.join(ROOT, "gps_logger.py")) as f:
    _lsrc = f.read()
_runs = _lsrc[_lsrc.index("def run():"):]
_loops = _lsrc[_lsrc.index("def _loop("):]
check("TFT-WIRING: only the Heltec starts it; run() starts it before the first GPS "
      "setup, the loop ticks it",
      _ft_seen == [] and gl._tft is None
      and _runs.index("_tft_start()") < _runs.index("gps_setup()")
      and "tft_tick(now)" in _loops, repr(_ft_seen))

# OTA: each board only its own firmware
_mf = {"build": 5, "commit": "abc1234", "size": 9, "sha256": "0" * 64,
       "url": "x.bin", "files": []}
_mfs = dict(_mf, board="GPS_LOGGER_S3")
check("OTA-BOARD: the Heltec takes only a manifest naming it, the C6 only one "
      "without a board (as published so far), and the name is signed",
      hl._ota_manifest_ok(_mfs) and not hl._ota_manifest_ok(_mf)
      and gl._ota_manifest_ok(_mf) and not gl._ota_manifest_ok(_mfs)
      and hl._ota_signed(_mfs) == gl._ota_signed(_mf) + b"|board=GPS_LOGGER_S3"
      and gl._ota_signed(_mf) == hl._ota_signed(_mf))
_uart_rx[:] = b""

print()
print("%d passed, %d failed" % (passed, failed))
if failures:
    print("failures:")
    for f in failures:
        print(" ", f)
sys.exit(1 if failed else 0)
