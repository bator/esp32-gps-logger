#!/usr/bin/env python3
"""CPython harness for gps_logger upload / crash / WiFi logic. Not deployed."""
from __future__ import print_function

import array
import atexit
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

# every temp directory the suite makes is removed when it ends (the test run
# used to leave about a dozen in /tmp each time); KEEP_TMP=1 keeps them to
# look at after a failure
_tmp_made = []
_mkdtemp0 = tempfile.mkdtemp


def _mkdtemp(*a, **k):
    d = _mkdtemp0(*a, **k)
    _tmp_made.append(d)
    return d


tempfile.mkdtemp = _mkdtemp


@atexit.register
def _tmp_clean():
    if os.environ.get("KEEP_TMP") != "1":
        for d in _tmp_made:
            shutil.rmtree(d, ignore_errors=True)


def _slurp(path):
    """A text file's content, closed."""
    with builtins.open(path) as f:
        return f.read()


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


def _load_web_min():
    """tools/web_min.py (without one: None, its checks fail, the rest runs)."""
    path = os.path.join(ROOT, "tools", "web_min.py")
    if not os.path.exists(path):
        return None
    spec = importlib.util.spec_from_file_location("web_min", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


web_min = _load_web_min()


def _page_min(raw):
    """web.html's bytes as they go on the board, before the gzip."""
    return web_min.minify(raw.decode("utf-8")).encode("utf-8") if web_min else raw


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
_resets = []
_posts = []
_post_code = [201]
_post_raise = [None]
_post_headers = [{}]
_wdt = []
_timers = []
_hostnames = []
_reset_cause = [1]
_uart_tx = []
_uart_rx = bytearray()
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


class _Pwm(object):
    """machine.PWM: every duty set lands in _pin_levels as (pin, duty_u16, clock), so
    a dark backlight reads (21, 0) as the Pin(value=0) before it did."""
    fail = False                                # PWM cannot be had
    live = {}                                   # pin number -> the PWM on it

    def __init__(self, pin, freq=0, duty_u16=0):
        if _Pwm.fail:
            raise ValueError("no PWM")
        self.n, self.freq, self.duty = pin.n, freq, duty_u16
        _Pwm.live[pin.n] = self
        _pin_levels.append((pin.n, duty_u16, _clock[0]))

    def duty_u16(self, d=None):
        if d is None:
            return self.duty
        self.duty = d
        _pin_levels.append((self.n, d, _clock[0]))
        return None


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


class _UcUart(UART):
    """The Heltec's UC6580 on the UART: answers each command ($FAIL for those
    in bad, nothing while silent), PDTINFO with what the board's receiver
    says."""
    PDT = b"$PDTINFO,UC6580I,G1B1L1E1,V00,R6.0.0.0Build3700,2400615000268,20230729000954003*64\r\n"

    def __init__(self, *a, bad=(), silent=False, **k):
        self.bad, self.silent, self.cmds = bad, silent, []

    def write(self, data):
        data = bytes(data)
        self.cmds.append(data)
        _uart_tx.append(data)
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


def FakeResp(code, headers=None):
    """What HTTP_POST returns: (code, Retry-After, Content-Type), the headers
    bytes as they came."""
    headers = headers or {}
    retry, ctype = headers.get("Retry-After"), headers.get("Content-Type")
    return code, retry and retry.encode(), ctype and ctype.encode()


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
    machine.SPI = _TftSpi
    machine.PWM = _Pwm
    machine.UART = _UcUart
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
    # a double keeps its digits in the departure ring, as the board's single
    # does in array("f")
    m._RF = array.array("d", bytes(8 * 3 * m.STILL_DEP_MAX))
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
_EPOCH_S = gl.EPOCH_DAYS * 86400         # 1970 -> the firmware's 2020

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


HEADER = "#TRK1\n"


def _kb_encode(lines, k, seg):
    """lines[:k] as the bytes of a .trb flush, as flush writes them a
    _kb_part at a time."""
    gl._kb_start()
    out = bytearray()
    while gl._kq[0] < k:
        out += gl._kb_part(lines, k, seg)
    return out


def _web_json():
    """The whole status.json as one string (the board streams the parts)."""
    return "".join(gl._web_json_parts(gl._web_rows(), gl._sv_list()))


def trk_line(lat, lon, t, ele="10.0", hdop=""):
    """One .trk point line; t is the ISO form, stored compact."""
    tc = t.replace("-", "").replace(":", "")
    return "%.6f,%.6f,%s,%s,%s\n" % (lat, lon, ele, hdop, tc)


def is_point(line):
    """One point line: "lat,lon,ele,hdop,time" (TRK1) or "lat,lon,ele,acc,speed,
    time" (TRK2). Without its newline (power cut mid-write) it is not a point."""
    return (line[-1:] == b"\n" and 4 <= line.count(b",") <= 5
            and line[:1] != b"#")


def pts_in(path):
    """Point lines in a track file."""
    with open(path, "rb") as f:
        return sum(1 for line in f if is_point(line))


def make_trb(path, n, times=None):
    """A .trb of n points, as flush writes it in one go."""
    ls = []
    for i in range(n):
        t = (times[i] if times else
             "2026-10-05T%02d:%02d:%02dZ" % (i // 3600, (i // 60) % 60, i % 60))
        ls.append(trk_line(52.5 + i * 0.0001, 13.4 + i * 0.0001, t))
    with builtins.open(path, "wb") as f:
        f.write(gl.HEADERB.encode() + bytes(_kb_encode(ls, n, None))
                + gl.FOOTER.encode())


def make_trk(path, n, extra=b"", times=None):
    parts = [HEADER.encode()]
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
            if n.endswith((".trk", ".trb", ".pos", ".tmp", ".bad")):
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


def _i7(v):
    """1e-7 degrees as "%.6f" prints them: an older NAV-PVT track's coordinates."""
    q = (v + 5) // 10 if v >= 0 else -((5 - v) // 10)
    if q < 0:
        q = -q
        return "-%d.%06d" % (q // 1000000, q % 1000000)
    return "%d.%06d" % (q // 1000000, q % 1000000)


def hist_add(lines, k):
    """lines[:k] into the day history as the logger does it: an uploaded .trb
    batch's (_hist_batch)."""
    tmp = os.path.join(diag_dir, "hist-batch.trb")
    with builtins.open(tmp, "wb") as f:
        f.write(gl.HEADERB.encode() + bytes(_kb_encode(lines, k, None))
                + gl.FOOTER.encode())
    gl._hist_batch(tmp, 0, os.path.getsize(tmp), None)
    os.remove(tmp)


def setup_cmds():
    """Setup commands the receiver got since _uart_tx was emptied."""
    return len([f for f in _uart_tx if f[:4] == b"$CFG"])


def settled():
    """The receiver past its UC_SETTLE: for checks that read a fix through the
    parser and are about something else (UC-SETTLE* test the rule itself)."""
    gl.st["settled"] = True


def locations():
    out = []
    for p in _posts:
        out.extend(json.loads(p["data"].decode())["locations"])
    return out


gl = _reload_logger()
gl.st.update(rx=None, valid=False, fixtype=None, quality=None)
gl.wifi.update(state="off", ssid=None)
gl.log.update(online=False)
gl.up.update(left=0)

# FLASH-CACHE: the free flash comes through _flash_kb's 30 s cache: statvfs walks
# the LittleFS, the page and the display ask often
_sv_calls = []
_real_statvfs = os.statvfs
os.statvfs = lambda p: _sv_calls.append(p) or _real_statvfs("/")
gl._fl[:] = [None, None]
try:
    _flash_vals = []
    for _ in range(6):
        _flash_vals.append(gl._flash_kb())
        _clock[0] += 500
    _clock[0] += 30000
    _flash_vals.append(gl._flash_kb())
finally:
    os.statvfs = _real_statvfs
check("FLASH-CACHE: _flash_kb reads statvfs once per 30 s, in kB",
      len(_sv_calls) == 2 and len(set(_flash_vals[:6])) == 1 and _flash_vals[0] > 0,
      "%r %r" % (_sv_calls, _flash_vals[:2]))

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


check("inf ele is not finite",
      not gl.math.isfinite(float("inf")))


tmpdir = tempfile.mkdtemp(prefix="gpxlog-")
try:
    # --- truncated last line ----------------------------------------------
    online(tmpdir)
    path = os.path.join(tmpdir, "20261005_100000.trk")
    make_trk(path, 3, extra=b'52.9,13.9,10.0,,20261005T11')
    n, n_ok, size, new_off, eof = gl.read_batch(path, 0)
    check("truncated: consumed 4 lines (3 good + 1 truncated)", n == 4)
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
    big = os.path.join(tmpdir, "20261005_110000.trk")
    make_trk(big, 350)
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

    ghost = os.path.join(tmpdir, "20261005_120000.trk.pos")
    with open(ghost, "w") as f:
        f.write("12\n")
    check("pending() ignores a planted .pos sidecar",
          ghost not in gl.pending() and all(not p.endswith(".pos") for p in gl.pending()))
    os.remove(ghost)

    # --- interrupted POST / offset commit (at-least-once, no loss) ---------
    online(tmpdir)
    path = os.path.join(tmpdir, "20261005_130000.trk")
    make_trk(path, 80)
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
    path = os.path.join(tmpdir, "20261005_131000.trk")
    make_trk(path, 20)
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
          gl.read_pos(path)[0] == 0)
    pump()
    check("powerloss: file is POSTed (not dropped)",
          len(_posts) == 1 and not os.path.exists(path),
          "posts=%s exists=%s state=%s" % (len(_posts), os.path.exists(path), gl.up["state"]))

    # --- corrupt .pos (past EOF) must reset to 0, never delete without POST
    online(tmpdir)
    path = os.path.join(tmpdir, "20261005_132000.trk")
    make_trk(path, 12)
    with open(path + ".pos", "w") as f:
        f.write("%d\n" % (os.path.getsize(path) + 500))
    check("corrupt pos: read_offset resets to 0",
          gl.read_pos(path)[0] == 0)
    pump()
    check("corrupt pos: POST happens (file not silently deleted)",
          len(_posts) == 1)
    check("corrupt pos: points uploaded",
          len(locations()) == 12)

    # empty / garbage .pos
    online(tmpdir)
    path = os.path.join(tmpdir, "20261005_133000.trk")
    make_trk(path, 4)
    with open(path + ".pos", "w") as f:
        f.write("not-a-number\n")
    check("garbage pos: treated as 0", gl.read_pos(path)[0] == 0)
    pump()
    check("garbage pos: uploaded", len(_posts) == 1)

    # --- HTTP 400: batch quarantine only -----------------------------------
    online(tmpdir)
    path = os.path.join(tmpdir, "20261005_140000.trk")
    make_trk(path, 10)
    _post_code[0] = 400
    pump()
    check("http400: batch parked in .trk.bad after one payload error",
          os.path.exists(path + ".bad"),
          "state=%s" % gl.up["state"])
    check("http400: original GPX gone (whole remaining file was the batch)",
          not os.path.exists(path))
    check("http400: pending() no longer lists the GPX",
          path not in gl.pending())

    # 80-point file: only first batch is parked, rest still uploads
    online(tmpdir)
    path = os.path.join(tmpdir, "20261005_141000.trk")
    make_trk(path, 80)
    _post_code[0] = 422
    pump()
    check("http422: GPX remains after batch quarantine",
          os.path.exists(path) and os.path.exists(path + ".bad"))
    with open(path + ".bad", "rb") as f:
        bad_n = sum(1 for line in f if is_point(line))
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
        path = os.path.join(tmpdir, "20261005_15%s.trk" % code)
        make_trk(path, 8)
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
    path = os.path.join(tmpdir, "20261005_15500.trk")
    make_trk(path, 5)
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
    path = os.path.join(tmpdir, "20261005_15401.trk")
    make_trk(path, 5)
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
    path = os.path.join(tmpdir, "20261005_16id.trk")
    make_trk(path, 3)
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
    path = os.path.join(tmpdir, "20261005_16bad.trk")
    with open(path, "wb") as f:
        f.write((HEADER + "52.5,13.4,10.0,,20261005T000000Z\n"
                 "52.5,13.4,inf,,20261005T000001Z\n"
                 '52.5,13.4,1.0,,20261005T000002Z"injected\n' + gl.FOOTER).encode())
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
    gl.log["file"] = os.path.join(tmpdir, "20261005_160000.trk")
    gl.log["created"] = False
    gl.log["batch"] = ["1.000000,2.000000,,,20261005T160000Z\n"]
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
    path = os.path.join(tmpdir, "20261005_170000.trk")
    make_trk(path, 40)
    # write a sane mid-file offset (start of a line)
    skipped = 0
    with open(path, "rb") as f:
        while skipped < 15:
            line = f.readline()
            if not line:
                break
            if is_point(line):
                skipped += 1
        mid = f.tell()
    gl.write_offset(path, mid)
    left = gl.count_points()
    check("count_points: counts from sidecar offset (not the whole file)",
          0 < left < 40, "left=%s mid=%s" % (left, mid))
    tmpf = os.path.join(tmpdir, "20261005_170000.trk.pos.tmp")
    with open(tmpf, "w") as f:
        f.write("1\n")
    gl.up_reset()
    check("up_reset: removes leftover .tmp sidecars",
          not os.path.exists(tmpf))

    orphan = os.path.join(tmpdir, "gone.trk.pos")
    with open(orphan, "w") as f:
        f.write("0\n")
    gl.up_reset()
    check("up_reset: removes orphan .trk.pos without a matching .trk",
          not os.path.exists(orphan))

    # --- WDT budget: count_points feeds inside the loop --------------------
    path = os.path.join(tmpdir, "20261005_17wdt.trk")
    make_trk(path, 200)
    dog = WDT(timeout=gl.WDT_TIMEOUT)
    gl._wdt = dog
    before = dog.fed
    gl.count_points()
    check("WDT budget: count_points feeds during a long read",
          dog.fed > before, "fed %s -> %s" % (before, dog.fed))
    online(tmpdir)
    make_trk(path, 200)
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
        check("crash handler: machine.reset() called (run() returned)", False, "run() returned")
    except SystemExit as e:
        check("crash handler: machine.reset() called",
              bool(_resets) and "reset" in str(e))
    except Exception as e:
        check("crash handler: machine.reset() called (other exception)", False, repr(e))
    finally:
        gl._loop = orig_loop

    # startup exception (before _loop) also resets
    _resets[:] = []
    gl._wdt = None
    orig_reset = gl.reset

    def boom_init():
        raise OSError("at start-up")

    gl.reset = boom_init
    try:
        gl.run()
        check("startup OSError: machine.reset() (run() returned)", False, "returned")
    except SystemExit:
        check("startup OSError: machine.reset()", bool(_resets))
    except Exception as e:
        check("startup OSError: machine.reset() (other exception)", False, repr(e))
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
        check("Ctrl-C: raised KeyboardInterrupt (a reset instead)", False, "reset")
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
    check("wifi: the status row shows SSID not password",
          dict(gl._web_rows())["wifi"] == "connected home")

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
    gl.wifi["nets"] = [{"ssid": "home", "ip": ("192.168.1.60", "255.255.255.0",
                                               "192.168.1.1", "192.168.1.1")},
                       {"ssid": "phone", "ip": None}]
    WLAN.ifconfig_calls = []
    gl._wifi_ip(WLAN(0), gl.wifi["nets"][1])
    check("static ip: leaving the static net hands the interface back to DHCP",
          WLAN.ifconfig_calls == ["dhcp"],
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
    path = os.path.join(tmpdir, "20261005_18bad.trk")
    make_trk(path, 4)
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
    path = os.path.join(tmpdir, "20261005_19e1.trk")
    make_trk(path, 2000)
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
            bad_n = sum(1 for line in f if is_point(line))
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
    path = os.path.join(tmpdir, "20261005_19e2.trk")
    make_trk(path, 10)
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
        still = sum(1 for line in f if is_point(line))
    check("B1 E2: quarantined points not deleted",
          still == 10, "still=%s" % still)

    # --- B1: .bad full: do not advance offset ------------------------------
    online(tmpdir)
    path = os.path.join(tmpdir, "20261005_19full.trk")
    make_trk(path, 80)
    with open(path + ".bad", "wb") as f:
        f.write(b"x" * (gl.MAX_BAD_BYTES))
    _post_code[0] = 400
    off0 = gl.read_pos(path)[0]
    pump()
    check("B1 bad-full: offset not advanced",
          gl.read_pos(path)[0] == off0 and os.path.exists(path),
          "off=%s state=%s" % (gl.read_pos(path)[0], gl.up["state"]))
    check("B1 bad-full: state is bad full",
          gl.up["state"] == "bad full")
    check("B1 bad-full: it stays so while the stuck file is there",
          gl._bad_full_left())
    later = os.path.join(tmpdir, "20261005_19later.trk")
    make_trk(later, 8)
    _posts[:] = []
    _post_code[0] = 201
    gl.up["next_at"] = _clock[0]
    pump()
    check("B1: another file's 201 does not clear BAD FULL",
          gl.up["state"] == "bad full",
          "state=%s sent=%s" % (gl.up["state"], gl.up["sent"]))
    check("B1: BAD FULL stays after another file succeeds",
          gl._bad_full_left())
    with open(path, "rb") as f:
        kept = sum(1 for line in f if is_point(line))
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
          not gl._bad_full_left() and gl.up["state"] != "bad full",
          "state=%s exists=%s" % (gl.up["state"], os.path.exists(path)))

    # --- B2: 200 kB line without newline must not MemoryError/boot-loop ----
    online(tmpdir)
    path = os.path.join(tmpdir, "20261005_20long.trk")
    with open(path, "wb") as f:
        f.write(HEADER.encode())
        f.write(b"52.500000,13.400000,,,20261005T000000Z\n")
        f.write(b"A" * 200000 + b"\n")
        f.write(b"52.500100,13.400100,,,20261005T000001Z\n")
        f.write(gl.FOOTER.encode())
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
        check("B2: count_points/up_reset catch MemoryError (it escaped)", False)
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
        check("B2: upload_tick MemoryError skips the file (it escaped)",
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

    current = gl.log.get("file")
    for n in os.listdir(tmpdir):
        p = os.path.join(tmpdir, n)
        if p == current:
            continue
        if n.endswith((".trk", ".trb", ".pos", ".tmp", ".bad")):
            try:
                os.remove(p)
            except OSError:
                pass
    finished = os.path.join(tmpdir, "20261004_210000.trk")
    make_trk(finished, 4)
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
    f1 = os.path.join(trk_dir, "flush.trb")
    p1 = trk_line(52.5, 13.4, "2026-10-05T12:00:05Z")
    p2 = trk_line(52.6, 13.5, "2026-10-05T12:00:06Z")
    p3 = trk_line(52.7, 13.6, "2026-10-05T12:00:07Z")
    gl.log.update(file=f1, batch=[p1, p2])
    gl.flush()
    gl.log["batch"] = [p3]
    gl.flush()
    with open(f1, "rb") as f:
        raw = f.read()
    with builtins.open(f1, "rb") as f:
        raw_b = f.read()
    check("TRK-3: flush writes the header, point records, FOOTER; appends keep the "
          "footer last", raw == (HEADER + p1 + p2 + p3 + gl.FOOTER).encode()
          and raw_b.startswith(gl.HEADERB.encode())
          and raw_b.endswith(gl.FOOTER.encode())
          and raw_b.count(gl.FOOTER.encode()) == 1, repr(raw))

    rb = os.path.join(trk_dir, "read.trk")
    with open(rb, "wb") as f:
        f.write((HEADER
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
    with open(os.path.join(cnt_dir, "old.gpx"), "wb") as f:
        f.write(b'<trkpt lat="52.5" lon="13.4"><time>2026-10-05T10:00:00Z</time></trkpt>\n')
    total = gl.count_points()
    check("TRK-5: count_points counts point lines of track files only",
          total == 3, "total=%s" % total)

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
        f.write((HEADER + "".join(good) + gl.FOOTER + "".join(drift)
                 + "52.6,13.5,,,2026100").encode())
    n, n_ok, size, new_off, eof = gl.read_batch(te, 0)
    te_left = gl.count_points()
    online(end_dir)
    with open(te, "wb") as f:
        f.write((HEADER + "".join(good) + gl.FOOTER + "".join(drift)
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
        f.write((HEADER + "".join(",".join(r) + "\n" for r in rows)
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
        f.write((HEADER + pt + fit + over + "x" * 3000 + "\n" + pt
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
        f.write((HEADER + "47.100000,19.100000,100.0,1e308,"
                 "20261101T100000Z\n" + gl.FOOTER).encode())
    n, n_ok, size, new_off, eof = gl.read_batch(inf_p, 0)
    j_trk = _strict_json(bytes(gl._body[:size]))
    check("UPL-5: hdop * HDOP_M not finite: no accuracy, valid JSON, the "
          "point kept",
          isinstance(j_trk, dict) and len(j_trk["locations"]) == 1
          and "horizontal_accuracy" not in j_trk["locations"][0]["properties"],
          repr(j_trk)[:300])

    # UPL-6: a point time that is no UTC time (month 13, day 32, Feb 30,
    # hour 24, minute 60, second 60) is no point, as _kb_time has it
    bad_t = ("20261301T100000Z", "20261032T100000Z", "20260230T100000Z",
             "20261101T240000Z", "20261101T106000Z", "20261101T100060Z",
             "20261100T100000Z", "20240229T100000Z", "20990101T000000Z")
    tm_p = os.path.join(end_dir, "20261005_180000.trk")
    with open(tm_p, "wb") as f:
        f.write((HEADER + "".join("47.1,19.1,1.0,1.0,%s\n" % t_
                                     for t_ in bad_t) + gl.FOOTER).encode())
    n, n_ok, size, new_off, eof = gl.read_batch(tm_p, 0)
    j_tm = _strict_json(bytes(gl._body[:size]))
    want_t = ["2024-02-29T10:00:00Z", "2099-01-01T00:00:00Z"]
    got_t = ([x["properties"]["timestamp"] for x in j_tm["locations"]]
             if isinstance(j_tm, dict) else j_tm)
    check("UPL-6: a point time out of range (month, day, hour, minute, "
          "second) is skipped",
          (n, n_ok) == (9, 2) and got_t == want_t, repr((n, n_ok, got_t)))

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
    boot_src = _slurp(boot_path)
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
    path = os.path.join(tmpdir, "20261005_22k4.trk")
    make_trk(path, 100)
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
            if is_point(line):
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
    path = os.path.join(tmpdir, "20261005_23ra.trk")
    make_trk(path, 5)
    _post_code[0] = 429
    _post_headers[0] = {"Retry-After": "-3"}
    pump()
    check("E4: negative Retry-After clamped to BACKOFF_START",
          gl.up["next_at"] == _clock[0] + gl.BACKOFF_START,
          "next_at=%s" % gl.up["next_at"])
    online(tmpdir)
    path = os.path.join(tmpdir, "20261005_24ra.trk")
    make_trk(path, 5)
    _post_code[0] = 503
    _post_headers[0] = {"Retry-After": "604800"}
    pump()
    check("E5: 7-day Retry-After clamped to BACKOFF_MAX",
          gl.up["next_at"] == _clock[0] + gl.BACKOFF_MAX,
          "next_at=%s" % gl.up["next_at"])
    _post_headers[0] = {}

    # --- orphan .trk.bad.pos -----------------------------------------------
    online(tmpdir)
    orphan_bad = os.path.join(tmpdir, "gone.trk.bad.pos")
    with open(orphan_bad, "w") as f:
        f.write("0\n")
    gl.up_reset()
    gl.up["url"] = "https://dawarich.example.com"
    gl.up["key"] = "secret-key"
    check("orphan .trk.bad.pos cleaned by up_reset",
          not os.path.exists(orphan_bad))

    # HOME1=0 override
    env_nh = "SSID=cafe\nWPA2=x\nHOME1=0\nGEO_URL=https://d.example\nAPI_KEY=k\n"
    w = start_wifi(env_nh, [("cafe", -40)], ["cafe"])
    check("HOME1=0: network #1 is not home",
          w["home"] is False, "home=%s" % w["home"])

    # --- REG1: no str.isalnum (MicroPython 1.29) ---------------------------
    src = _slurp(os.path.join(ROOT, "gps_logger.py"))
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
    stuck = os.path.join(tmpdir, "20261001_000000.trk")
    later = os.path.join(tmpdir, "20261005_000000.trk")
    make_trk(stuck, 200)
    _post_code[0] = 400
    pumps = 0
    while stuck not in (gl.up.get("hold") or {}) and pumps < 8:
        pump()
        gl.up["next_at"] = _clock[0]
        pumps += 1
    check("HOL: oldest file is held after global payload errors",
          stuck in (gl.up.get("hold") or {}),
          "hold=%s pumps=%s" % (gl.up.get("hold"), pumps))
    make_trk(later, 8)
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
    make_trk(open_path, 3)
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
        with open(path, "rb") as f:             # (a .trb as its text)
            return sum(1 for line in f if is_point(line))

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
    close_path = os.path.join(close_dir, "c.trb")
    make_trb(close_path, 2)
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
    make_trb(close_path, 2)
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
    new_path = os.path.join(close_dir, "new.trb")
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

    part_path = os.path.join(close_dir, "part.trb")
    make_trb(part_path, 2)
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

    create_part = os.path.join(close_dir, "create_part.trb")
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
          and create_body.startswith(gl.HEADERB.encode())
          and create_body.endswith(gl.FOOTER.encode()),
          "created=%s n_pts=%s head=%r" % (
              gl.log.get("created"), _gpx_pts(create_part), create_body[:80]))

    # M7: transient stat/open EIO must not flip created=False (wb would wipe)
    eio_path = os.path.join(close_dir, "eio.trb")
    make_trb(eio_path, 112)
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

    gone_path = os.path.join(close_dir, "gone.trb")
    make_trb(gone_path, 2)
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

    trunc_path = os.path.join(close_dir, "trunc.trb")
    make_trb(trunc_path, 2)
    with orig_open(trunc_path, "rb") as f:
        trunc_raw = f.read()
    with orig_open(trunc_path, "wb") as f:
        f.write(trunc_raw[:-3])
    gl.log.update(file=trunc_path, created=True, points=2, batch=[extra_pt],
                  err=0, new_seg=False, good_size=len(trunc_raw))
    gl.flush()
    wf10 = gl.log["file"]
    gl.flush()
    gl.log.update(file=None, created=False)
    check("WF-10: a footer not where it was: no append after a guess, the batch goes "
          "on in a new file",
          wf10 != trunc_path and _gpx_pts(wf10) == 1 and gl.log["batch"] == []
          and _gpx_pts(trunc_path) == 2,
          "%r n=%s" % (wf10, _gpx_pts(trunc_path)))

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
    empty_bad = os.path.join(tmpdir, "empty.trk.bad")
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

    eof_bad = os.path.join(tmpdir, "tail.trk.bad")
    with open(eof_bad, "wb") as f:
        f.write(b"52.500000,13.400000,,,20261005T000000Z\n")
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
    boot_src = _slurp(boot_path)
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
    src = _slurp(os.path.join(ROOT, "gps_logger.py"))
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
              gl._recent(now, gl.wifi["failed"].get("home"), gl.FAIL_HOLD)
              is False)
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
    # reckoning, the last fix carried on) and were a fix. DGPS (2) still is
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
        settled()
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
        settled()
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
        settled()
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

    # reset loop: restarts over and over -> warned and counted
    out = []
    real_print = gl.print if hasattr(gl, "print") else None
    gl.print = lambda *a, **k: out.append(" ".join(str(x) for x in a))
    try:
        banner = b"FWVer R6.0.0.0Build3700\r\n"
        gl.reset()
        _uart_rx.extend(banner)
        for k in range(12):
            _uart_rx.extend(nmea("GPRMC,1205%02d.00,V,,,,,,,171026,,,N" % k)
                            + b"\r\n")
            gl.read_uart()
            _clock[0] += 1000
            gl.gps_check(_clock[0])
        normal = gl.st["loops"]
        # restarts with NMEA in between: GPS_FLAP_RESTARTS in the window
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

    import inspect
    run_src = inspect.getsource(gl.run)
    check("HOLD: Ctrl-C / crash flushes keep the held drift out",
          run_src.count("flush(_still_hold())") == 2   # crash, finally (Ctrl-C)
          and "flush()" not in run_src)

    # GST: the receiver's own error estimate keeps a converging cold start
    # out (good HDOP, hundreds of metres off), a settled fix is used
    def gst_second(hhmmss, std):
        epoch(hhmmss)
        gl.handle(nmea("GPGST,%s,20,,,,%.1f,%.1f,%.1f"
                       % (hhmmss, std, std * 0.8, std * 2)))

    gl.reset()
    settled()
    gst_second("120100.00", 150.0)
    epoch("120101.00")
    check("GST: an estimate of 150 m makes the second unusable",
          gl.st["std"] == 150.0 and not gl._usable(_clock[0])
          and gl._sample_line(_clock[0], None, None) is None)
    gl.reset()
    settled()
    gst_second("120110.00", 4.0)
    epoch("120111.00")
    check("GST: an estimate of 4 m is usable",
          gl.st["std"] == 4.0 and gl._usable(_clock[0])
          and gl._sample_line(_clock[0], None, None) is not None)
    gl.reset()
    settled()
    epoch("120120.00")
    epoch("120121.00")
    check("GST: without GST (none yet, or its setup lost) only HDOP decides",
          gl.st["std"] is None and gl._usable(_clock[0]))
    # REV-5: at 1 Hz GST comes once per 5 epochs (and one may be lost):
    # the seconds between keep its estimate for GST_HOLD, not None (that
    # let 4 of 5 seconds of a converging cold start through)
    gl.reset()
    settled()
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

    # SETUP-AGAIN: a receiver still booting at setup stays at 1 Hz -> sent
    # again, less and less often; one that took it is left alone
    gsv_line = nmea("GPGSV,3,1,12,01,40,083,46,02,17,308,41,12,07,344,39,"
                    "14,22,228,45,1")
    vtg_line = nmea("GNVTG,,T,,M,0.0,N,0.0,K,A")
    cfg_n = setup_cmds

    gl.reset()
    _uart_rx[:] = []
    _uart_tx[:] = []
    gl.gps_setup()
    gl.read_uart()
    n_frames = cfg_n()
    gl.handle(vtg_line)                             # still in the UART buffer
    gl.handle(gsv_line)                             # (GSV stays on: never extra)
    _clock[0] += gl.GPS_RETRY
    gl.gps_check(_clock[0])
    check("SETUP-AGAIN: sentences queued before the setup do not count",
          cfg_n() == n_frames and gl.st["extra"] == 0)
    gl.handle(gsv_line)
    gl.gps_check(_clock[0])
    no_again = cfg_n()
    gl.handle(vtg_line)
    gl.gps_check(_clock[0])
    check("SETUP-AGAIN: VTG after the setup -> it is sent again; GSV is not "
          "what it turned off",
          no_again == n_frames and cfg_n() == 2 * n_frames
          and gl.st["extra"] == 0 and gl.st["setup_wait"] == gl.GPS_RETRY,
          "%d %d %d" % (no_again, cfg_n(), n_frames))
    _clock[0] += 1500
    gl.handle(nmea("GNGLL,,,,,120000.00,V,N"))
    _clock[0] += gl.GPS_RETRY
    gl.gps_check(_clock[0])
    check("SETUP-AGAIN: one that acknowledges and ignores (GLL too): again after "
          "GPS_RETRY, then doubling",
          cfg_n() == 3 * n_frames and gl.st.get("ack_n") == 2
          and gl.st["setup_wait"] == 2 * gl.GPS_RETRY,
          "frames=%d wait=%d" % (cfg_n(), gl.st["setup_wait"]))
    _clock[0] += 10 * gl.GPS_RETRY_MAX
    gl.gps_check(_clock[0])
    check("SETUP-AGAIN: a receiver that took it is never sent it again",
          cfg_n() == 3 * n_frames)
    # a receiver that never answers: less and less often
    gl.uart.silent = True
    try:
        gl.reset()
        gl.gps_setup()
        sends = []
        for _ in range(40):
            _clock[0] += 1500
            gl.handle(gsv_line)
            n0 = cfg_n()
            gl.gps_check(_clock[0])
            if cfg_n() > n0:
                sends.append(_clock[0])
        gaps = [(b2 - a2) // 1000 for a2, b2 in zip(sends, sends[1:])]
    finally:
        gl.uart.silent = False
    check("SETUP-AGAIN: a silent receiver is retried with a doubling wait",
          len(gaps) >= 2 and gaps == sorted(gaps) and gaps[-1] > gaps[0],
          "gaps=%s" % gaps)
    # a restart banner sets it up again within GPS_RETRY
    gl.reset()
    _uart_rx[:] = []
    gl.gps_setup()
    gl.read_uart()
    gl.st["setup_wait"] = gl.GPS_RETRY_MAX
    n0 = cfg_n()
    _clock[0] += gl.GPS_RETRY
    gl.handle(b"FWVer R6.0.0.0Build3700")
    gl.gps_check(_clock[0])
    check("SETUP-AGAIN: a restart banner sends the setup again at once",
          cfg_n() > n0 and gl.st["gps_boots"] == 1)
    # a receiver that stops answering mid-setup: the rest not sent, no long block
    gl.reset()
    _uart_rx[:] = []
    _uart_tx[:] = []
    real_write = _UcUart.write
    calls = [0]

    def silent_after_two(self, data):
        """Answer only PDTINFO and the first setup command."""
        calls[0] += 1
        if calls[0] <= 2:
            return real_write(self, data)
        _uart_tx.append(bytes(data))
        return len(data)
    _UcUart.write = silent_after_two
    t0 = _clock[0]
    try:
        gl.gps_setup()
    finally:
        _UcUart.write = real_write
    check("SETUP-AGAIN: a receiver silent after the first command: setup_ok "
          "False, the rest not sent, one UC_ACK_WAIT",
          gl.st["setup_ok"] is False and cfg_n() == 2
          and _clock[0] - t0 <= gl.UC_ACK_WAIT + 100,
          "ms=%d frames=%d" % (_clock[0] - t0, cfg_n()))
    _uart_rx[:] = []
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
              + nmea("GNGSA,A,3,01,02,03,,,,,,,,,,2.0,1.0,1.7,1"))
    check("HANDLE: garbage (0xB5 0x62 too) that runs into a line does not lose it",
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
        fs = gl._fix_s()                 # (none outside 2020-2099: _ymd_ok)
        ok = ok and (fs is None if y in (2000, 2100) else fs + _EPOCH_S == secs) \
            and gl._stamp_s(secs - _EPOCH_S)[0] == iso
    ok = ok and gl._stamp_s(calendar.timegm((2026, 12, 31, 23, 59, 59, 0, 0, 0)) - _EPOCH_S
                             + 1)[0] == "2027-01-01T00:00:00Z"
    check("HB: UTC date arithmetic matches the calendar (year end, leap, 2100)",
          ok)
    # the clock counts from 2020: a small int on MicroPython (< 2**30)
    gl.st.update(date="2026-10-18", time="08:00:00")
    s20 = gl._fix_s() if hasattr(gl, "_fix_s") else None
    check("HB: internal seconds count from 2020 and stay below 2**30",
          s20 is not None and 0 < s20 < 2 ** 30
          and gl._stamp_s(s20)[0] == "2026-10-18T08:00:00Z"
          and gl._fix_s() + _EPOCH_S == calendar.timegm((2026, 10, 18, 8, 0, 0, 0, 0, 0)),
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
    saved_aps = WLAN.scan_aps
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
        WLAN.scan_aps = saved_aps
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
        WLAN.scan_aps = saved_aps
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
    settled()
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
          not gl.has_fix(_clock[0]) and gl._fix_s() is None
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
          and not wl().active() and gl.wifi["state"] != "connected",
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
          "web_tick" in inspect.getsource(gl._loop)
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

    # SATS: the UC6580's satellite list: a GSV round, GSA's used ones, read out
    # of the stream; status.json lists it with names per system
    def sats_round():
        """A round's sentences, then the next round's first that closes it."""
        for s_ in ("GNGSA,A,3,12,05,,,,,,,,,,,1.2,0.8,0.9,1",
                   "GNGSA,A,3,11,,,,,,,,,,,,1.2,0.8,0.9,3",
                   "GNGSA,A,3,22,,,,,,,,,,,,1.2,0.8,0.9,4",
                   "GPGSV,1,1,03,05,71,200,44,12,54,010,38,25,08,300,,1",
                   "GAGSV,1,1,01,11,45,120,37,7",
                   "GBGSV,1,1,02,22,20,010,30,33,,,,1",
                   "GLGSV,1,1,01,03,05,300,,1",
                   "GPGSV,1,1,00,1"):
            _uart_rx.extend(nmea(s_) + b"\r\n")

    gl.reset()
    _uart_rx[:] = []
    gl.st["setup_ok"] = True
    sats_round()
    t0 = _clock[0]
    gl.read_uart()
    c = FakeConn(b"GET /status.json HTTP/1.1\r\n\r\n")
    gl._web_reply(c, c.req)
    waited = _clock[0] - t0
    jd = json.loads(c.sent.partition(b"\r\n\r\n")[2])
    names = [(v["name"], v["used"], v["elev"]) for v in jd["satellites"]]
    check("SATS: a GSV round lists the satellites per system, used from GSA, "
          "used first, by signal; 0x0A-free text read out of the stream",
          gl.st["err"] == 0 and names == [
              ("GPS 5", True, 71), ("GPS 12", True, 54), ("Galileo 11", True, 45),
              ("BeiDou 22", True, 20), ("GPS 25", False, 8),
              ("BeiDou 33", False, None), ("GLONASS 3", False, 5)]
          and jd["satellites_age_s"] == 0 and waited == 0,
          repr((gl.st["err"], names, waited)))
    check("SATS: status.json's row of a satellite",
          jd["satellites"][1] == {"name": "GPS 12", "system": "GPS", "svid": 12,
                                  "cno": 38, "elev": 54, "azim": 10, "used": True,
                                  "unhealthy": False},
          repr(jd["satellites"][1]))
    check("SATS: seen counts the satellites with a signal, used those in the fix",
          gl.sv_seen() == 4 and gl.sv_used() == 4,
          repr((gl.sv_seen(), gl.sv_used())))
    gl.handle(b"FWVer R6.0.0.0Build3700")
    check("SATS: a receiver restart forgets the list and the model",
          (gl.st["svs"], gl.st["gps_ver"]) == (None, None),
          repr((gl.st["svs"], gl.st["gps_ver"])))
    gl.reset()
    gl.handle(gsv_line)
    seen_gsv = gl.sv_seen()
    _uart_rx[:] = []
    gl.gps_setup()
    check("SATS: GSV's count until a list is made; a setup leaves it (GSV stays on)",
          seen_gsv == 12 and gl.st["view"] == {"GP": 12} and gl.sv_seen() == 12,
          "%d %r" % (seen_gsv, gl.st["view"]))
    gl.reset()
    _uart_rx[:] = []
    t0 = _clock[0]
    c3 = FakeConn(b"GET /status.json HTTP/1.1\r\n\r\n")
    gl._web_reply(c3, c3.req)
    jd3 = json.loads(c3.sent.partition(b"\r\n\r\n")[2])
    check("SATS: no list yet: empty, no age, and the page does not wait for one",
          jd3["satellites"] == [] and jd3["satellites_age_s"] is None
          and _clock[0] == t0, repr((jd3["satellites"], _clock[0] - t0)))

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
        try:
            gl._freq_bad[:] = []
            gl._set_freq(100)
            gl._set_freq(100)
            once = sum(1 for o in out if o.startswith("cpu freq 100")) == 1
            s3 = [gl._mhz_env({"CPU_MHZ": v}, "CPU_MHZ", 80)
                  for v in ("40", "240", "160", "80", "0", "100")]
        finally:
            if real_print is None:
                del gl.print
            else:
                gl.print = real_print
        check("CPU: an unsupported clock is reported once, not per POST",
              once, repr(out))
        check("CPU: 20/40/80/160/240 (or 0) are accepted from .env",
              s3 == [40, 240, 160, 80, 0, 80], repr(s3))
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

    # PB_KEEPALIVE: with the radio asleep, one scan every PB_KEEPALIVE ms keeps a
    # power bank from switching off; none while the radio is on anyway
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
    try:
        sv_reset()
        w, calls, out = rv_start_wifi(rv_env, [("home", -40)], ["home"])
        for _ in range(3):
            _clock[0] += gl.PB_KEEPALIVE
            gl.keepalive_tick(_clock[0])
        check("PB_KEEPALIVE: no keep-alive pulse while connected",
              gl.wifi["state"] == "connected" and gl.wifi["pb_scans"] == 0)
        gl._wifi_off(wl(), "sleep")
        n0 = scans[0]
        for _ in range(4):
            _clock[0] += gl.PB_KEEPALIVE // 2
            gl.keepalive_tick(_clock[0])
        check("PB_KEEPALIVE scan: one scan per PB_KEEPALIVE, radio off after",
              scans[0] - n0 == 2 and gl.wifi["pb_scans"] == 2
              and not wl().active() and gl.wifi["state"] == "sleep",
              "scans=%d" % (scans[0] - n0))
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
                   dict(gl._web_rows())["wifi"])
        except Exception as e:
            crash = e
        finally:
            WLAN.scan = real_scan
            gl.wifi_reset()
    check("WR-11: a non-UTF-8 .env neither raises nor loses the network",
          ok_env and got is not None and got[0] == "connected"
          and got[1] == [(raw, b"jelsz\xf3")]
          and got[2].startswith("connected K"), "%r %r %r" % (crash, env, got))

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
lb_plain = os.path.join(lb_td, "web.min.html")
with open(lb_plain, "wb") as f:
    f.write(_page_min(lb_page))
with open(lb_gz, "wb") as f:              # (as deploy.sh makes it)
    f.write(_gz_small(_page_min(lb_page), getattr(gl, "WEB_GZ_WBITS", 12)))
for interp, tag in ((mpy_bin, "MicroPython unix"), (sys.executable, "CPython")):
    if not interp:
        print("SKIP  WEB-LOOPBACK on %s (no binary)" % tag)
        continue
    try:
        out = subprocess.check_output(
            [interp, os.path.join(ROOT, "tests", "web_loopback.py"), lb_gz, lb_plain],
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
import os, sys
a = sys.argv[1:]
if a[:1] == ["-c"]:
    sys.exit(0)
if a and a[0].endswith("web_min.py"):      # the page's gzip: the real tool
    os.execv(sys.executable, [sys.executable] + a)
if os.environ.get("FAKE_PY_LOG"):
    with open(os.environ["FAKE_PY_LOG"], "a") as f:
        f.write(" ".join(a) + "\n")
out = a[a.index("-o") + 1]
with open(a[-1], "rb") as f, open(out, "wb") as g:
    g.write(b"MPY" + f.read())
"""
FAKE_SHIM = r"""
import sys, os, types
sys.implementation._machine = os.environ.get(
    "FAKE_MACHINE", "Heltec Wireless Tracker with ESP32S3")
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
    for n in ("deploy.sh", "gps_logger.py", "main.py", "web.html", "gps_boot.py",
              "gps_tft.py", "gps_tft_font.py"):
        shutil.copy(os.path.join(ROOT, n), work)
    os.makedirs(os.path.join(work, "tools"))
    if web_min:
        shutil.copy(os.path.join(ROOT, "tools", "web_min.py"),
                    os.path.join(work, "tools"))
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
        with open(os.path.join(work, "gps_boot.py"), "rb") as f:
            boot_sha = hashlib.sha256(f.read()).hexdigest()
        tft_sha = hashlib.sha256()
        for n in ("gps_tft.py", "gps_tft_font.py"):
            with open(os.path.join(work, n), "rb") as f:
                tft_sha.update(f.read())
        with open(os.path.join(dev, "gps_frozen.py"), "w") as f:
            f.write('COMMIT = "%s"\n' % {"quote": "ab'c", "dirty": "abc1234-dirty"}.get(
                fz, "abc1234"))                 # "old": no SHA (first builds)
            if fz != "old":
                f.write('SHA = "%s"\n' % (sha if fz != "stale" else "0" * 64))
            if fz in ("full", "tft-stale", "boot-stale", "quote", "dirty"):  # a marker with the
                f.write('TFT = "%s"\n' % (tft_sha.hexdigest()  # display's and
                                          if fz != "tft-stale" else "0" * 64))
                f.write('BOOT = "%s"\n' % (boot_sha           # gps_boot.py's SHA
                                           if fz != "boot-stale" else "0" * 64))
    env = dict(os.environ, PORT="/dev/fakeACM0", FAKE_DEV=dev,
               FAKE_LOG=os.path.join(work, "log"),
               FAKE_LOGGER=os.path.join(work, "logger"),
               FAKE_SHIM=os.path.join(work, ".venv", "bin", "shim.py"))
    env.update(env_extra)
    cwd = None
    if env_extra.get("ALT_VENV"):               # VENV=venv, relative to where it is
        alt = os.path.join(work, "alt")         # run (not to the repo): the venv
        os.makedirs(alt)                        # is moved there, .venv is gone
        shutil.move(os.path.join(work, ".venv"), os.path.join(alt, "venv"))
        env.update(VENV="venv",
                   FAKE_SHIM=os.path.join(alt, "venv", "bin", "shim.py"))
        cwd = alt
    r = subprocess.run(["bash", os.path.join(work, "deploy.sh")], env=env,
                       capture_output=True, text=True, timeout=60, cwd=cwd)
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
        new_page = _page_min(f.read())
    rc, out, err, dev, calls = deploy_run({})
    cps = [c for call in calls for c in call if c[0] == "cp"]
    check("DEPLOY: copies to .dpl, checks, renames; old .dpl and stray .py go",
          rc == 0 and dev_file(dev, "gps_logger.mpy") == new_mpy
          and dev_file(dev, "gps_logger.py") is None
          and not [n for n in os.listdir(dev) if n.endswith(".dpl")]
          and dev_file(dev, ".rtc") == b"0"
          and len(cps) == 3 and all(c[2].endswith(".dpl") for c in cps)
          and "Board: Heltec Wireless Tracker" in out and "(xtensawin)" in out
          and "Deployed to /dev/fakeACM0" in out,
          "rc=%d %r %r %r" % (rc, out[-300:], err[-300:], cps))
    dp_w = getattr(gl, "WEB_GZ_WBITS", 12)
    check("DEPLOY-GZIP: only web.html.gz goes up, stripped (tools/web_min.py), "
          "made with the board's small window (WEB_GZ_WBITS: it inflates it for "
          "curl), no mtime; the old plain web.html is removed in the same swap",
          _gunzip_w(dev_file(dev, "web.html.gz") or b"", dp_w) == new_page
          and (dev_file(dev, "web.html.gz") or b"")[4:8] == b"\0\0\0\0"
          and ["cp", "web.html.gz", ":web.html.gz.dpl"] in cps
          and not [c for c in cps if c[1] == "web.html"]
          and dev_file(dev, "web.html") is None, repr(cps))
    # DEPLOY-OPT3: both ways deploy.sh compiles the logger leave the line
    # numbers out, as the frozen firmware does (108097 -> 97980 B on xtensawin);
    # -O3 would drop an assert or a __debug__ branch too: there must be none
    dp_opt = []
    for extra in ({}, {"FAKE_FROZEN": "1", "FORCE_MPY": "1"}):
        pl = os.path.join(tempfile.mkdtemp(dir=dp_td), "py.log")
        deploy_run(dict(extra, FAKE_PY_LOG=pl))
        with open(pl) as f:
            dp_opt.append(f.read().split("\n")[:-1])
    opt_src = {}
    for n in ("gps_logger.py", "gps_boot.py", "gps_tft.py"):
        with open(os.path.join(ROOT, n)) as f:
            opt_src[n] = [ln for ln in f.read().splitlines()
                          if re.match(r"\s*assert\b", ln) or "__debug__" in ln]
    check("DEPLOY-OPT3: deploy.sh compiles with -O3, plain and FORCE_MPY; no "
          "assert or __debug__ in what is compiled that way",
          len(dp_opt) == 2 and all(len(c) == 1 and "-O3" in c[0].split()
                                   for c in dp_opt)
          and not any(opt_src.values()), repr((dp_opt, opt_src)))
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
    # DEPLOY-BOOT: the marker's BOOT (gps_boot.py: main.py's OTA and boot-retry
    # helpers, frozen) and TFT (the display's modules) are compared too, each
    # named when stale; a marker that has them all matching deploys. (A
    # marker without them, an older firmware, is the "match" case above.)
    rc_b, out_b, err_b, dev_b, calls_b = deploy_run({"FAKE_FROZEN": "boot-stale"})
    rc_t, out_t, err_t, dev_t, calls_t = deploy_run({"FAKE_FROZEN": "tft-stale"})
    rc_f, out_f, err_f, dev_f, calls_f = deploy_run({"FAKE_FROZEN": "full"})
    check("DEPLOY-BOOT: a frozen gps_boot.py (or display module) not this one "
          "stops like a stale logger, naming it; SHA, TFT and BOOT matching "
          "deploys main.py and the page",
          rc_b != 0 and "gps_boot.py" in err_b
          and "tools/build_firmware.sh --flash" in err_b
          and not [c for call in calls_b for c in call if c[0] == "cp"]
          and dev_file(dev_b, "main.py") == b"old main"
          and rc_t != 0 and "gps_tft.py" in err_t
          and dev_file(dev_t, "main.py") == b"old main"
          and rc_f == 0 and "Deployed" in out_f
          and dev_file(dev_f, "main.py") != b"old main"
          and dev_file(dev_f, "gps_logger.mpy") is None
          and sorted(c[1] for call in calls_f for c in call if c[0] == "cp")
          == ["main.py", "web.html.gz"],
          "%r %r %r" % ((rc_b, err_b[-300:]), (rc_t, err_t[-200:]),
                        (rc_f, out_f[-200:], err_f[-300:])))
    # DEPLOY-RT2 (RT2-5): the marker's words go to the shell only as words: a
    # quote in the commit is no answer (it used to be spliced into the Python
    # run on the board, a SyntaxError after the copy); a "-dirty" commit is one;
    # a relative VENV is the caller's (it was the repo's, after the cd)
    rc_q, out_q, err_q, dev_q, calls_q = deploy_run({"FAKE_FROZEN": "quote"})
    rc_d, out_d, err_d, dev_d, calls_d = deploy_run({"FAKE_FROZEN": "dirty"})
    rc_v, out_v, err_v, dev_v, calls_v = deploy_run({"FAKE_FROZEN": "full",
                                                     "ALT_VENV": "1"})
    check("DEPLOY-RT2: a quote in the frozen marker's commit -> 'Could not "
          "tell', nothing copied; a -dirty commit deploys, the shadowing "
          "gps_logger.mpy removed; VENV=venv relative to the caller's cwd works",
          rc_q != 0 and "Could not tell" in err_q
          and not [c for call in calls_q for c in call if c[0] == "cp"]
          and dev_file(dev_q, "main.py") == b"old main"
          and rc_d == 0 and "Deployed" in out_d
          and dev_file(dev_d, "gps_logger.mpy") is None
          and dev_file(dev_d, "main.py") != b"old main"
          and rc_v == 0 and "Deployed" in out_v
          and dev_file(dev_v, "main.py") != b"old main",
          "%r %r %r" % ((rc_q, err_q[-200:]), (rc_d, err_d[-200:]),
                        (rc_v, out_v[-100:], err_v[-300:])))
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
    # DEPLOY-BOARD: the logger runs on the Heltec alone: a C6 or a plain ESP32
    # is refused before anything is copied
    db = []
    for mach in ("Generic ESP32C6 module with ESP32C6", "Generic ESP32 module with ESP32",
                 "Raspberry Pi Pico with RP2040"):
        rc, out, err, dev, calls = deploy_run({"FAKE_MACHINE": mach})
        db.append((rc != 0, "Unsupported board" in err,
                   not [c for c in calls if "cp" in [x[0] for x in c]]))
    rc, out, err, dev, calls = deploy_run({})
    check("DEPLOY-BOARD: anything but the Heltec is refused, nothing copied",
          db == [(True, True, True)] * 3 and rc == 0, repr(db))
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
    print("Detected flash size: " + os.environ.get("FAKE_FLASH", "8MB"))
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
    if web_min:
        shutil.copy(os.path.join(ROOT, "tools", "web_min.py"),
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
               BOARD_DIR=os.path.join(ROOT, "boards", "GPS_LOGGER_S3"),
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
    man_re = r'module\("(\w+)\.py", base_path="[^"]*"(, opt=3)?\)'
    rc_s3, _o, _e, work_s3, _c = build_run(
        [], {"BOARD_DIR": os.path.join(ROOT, "boards", "GPS_LOGGER_S3")})
    with open(os.path.join(work_s3, "firmware", "manifest.py")) as f:
        man_s3 = re.findall(man_re, f.read())
    opt3 = [(n, ", opt=3") for n in ("gps_logger", "gps_boot", "gps_tft",
                                     "gps_tft_font")]
    check("BUILD-OPT3: the frozen modules with code leave the line numbers out "
          "(opt=3: ~10 KB of the logger), the display's too",
          rc == 0 and rc_s3 == 0 and all(x in man_s3 for x in opt3)
          and ("gps_frozen", "") in man_s3, repr(man_s3))
    bd = os.path.join(ROOT, "boards", "GPS_LOGGER_S3")
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
    # --migrate: only a board of its flash size; backup, erase, write the whole build
    rc, out, err, work, calls = build_run(["--migrate"], {"FAKE_FLASH": "4MB"})
    m4 = (rc, [c for c in calls if "erase_flash" in c or "write_flash" in c])
    rc, out, err, work, calls = build_run(["--migrate"], {})
    mig = [("read 0" if "read_flash" in c and c[c.index("read_flash") + 1]
            == "0" else "erase" if "erase_flash" in c else "write" if
            "write_flash" in c else "id" if "flash_id" in c else c[1])
           for c in calls]
    check("BUILD-MIGRATE: refused on another flash size; else flash id, backup of the "
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
    # fails and a board of the right size was refused
    rc, out, err, work, calls = build_run(["--migrate"],
                                          {"FAKE_FLASH_NOISE": "1"})
    check("BUILD-MIGRATE-EPIPE: more esptool output after the flash size "
          "line does not refuse the board",
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
    os.chmod(key_file, 0o600)
    os.chmod(bad_key, 0o600)
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
    # BUILD-PUBLISH-RT2 (RT2-6): the key file's mode (everyone: refused, the
    # group: warned, the key never printed) and OTA_DEST's directory (a plain
    # path only), both before any upload; the second also before the build
    key_secret = "0123456789abcdef0123456789abcdef"
    k644, k640 = (os.path.join(key_dir, n) for n in ("k644.key", "k640.key"))
    for kf, mode in ((k644, 0o644), (k640, 0o640)):
        with open(kf, "w") as f:
            f.write(key_secret + "\n")
        os.chmod(kf, mode)
    dest_k = tempfile.mkdtemp(dir=dp_td)
    rc_k4, out_k4, err_k4, _w, calls_k4 = build_run(
        ["--publish"], {"OTA_KEY_FILE": k644, "OTA_DEST": "fakehost:" + dest_k})
    rc_k0, out_k0, err_k0, _w, calls_k0 = build_run(
        ["--publish"], {"OTA_KEY_FILE": k640, "OTA_DEST": "fakehost:" + dest_k})
    bad_dests = {}
    for d_ in ("~/ota", "/srv/my ota", "/srv/$x", "/srv/a;b", "/srv/a'b"):
        bad_dests[d_] = build_run(["--publish"], {"OTA_KEY_FILE": key_file,
                                                  "OTA_DEST": "fakehost:" + d_})
    bad_dests["-oProxyCommand=x:/d"] = build_run(
        ["--publish"], {"OTA_KEY_FILE": key_file,
                        "OTA_DEST": "-oProxyCommand=x:/d"})
    check("BUILD-PUBLISH-RT2: a key file readable by everyone (644) is refused "
          "before any upload, one readable by its group (640) is published "
          "with a warning, the key itself never printed; an OTA_DEST "
          "directory with ~, a space or a shell special (or a host starting "
          "with a dash) is refused before the build and any scp or ssh",
          rc_k4 != 0 and "readable by everyone" in err_k4
          and not [c for c in calls_k4 if c[0] in ("scp", "ssh")]
          and rc_k0 == 0 and "WARNING" in err_k0 and "group" in err_k0
          and os.path.exists(os.path.join(dest_k, "manifest.json"))
          and key_secret not in out_k4 + err_k4 + out_k0 + err_k0
          and all(r_[0] != 0 and "OTA_DEST" in r_[2] and "Built" not in r_[1]
                  and not [c for c in r_[4] if c[0] in ("scp", "ssh")]
                  for r_ in bad_dests.values()),
          repr((rc_k4, err_k4[-200:], rc_k0, err_k0[-200:],
                {k_: (r_[0], r_[2][-120:]) for k_, r_ in bad_dests.items()})))
    # BUILD-PUBLISH-BOARD: the board names itself in the manifest (signed); a
    # folder holding another board's manifest (an older board's had no name) is
    # refused
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
    dest_c6 = tempfile.mkdtemp(dir=dp_td)
    with open(os.path.join(dest_c6, "manifest.json"), "w") as f:
        json.dump({"build": 1, "commit": "abc1234", "url": "x.bin"}, f)
    rc_onc6, _o, err_c6, _w, _c = build_run(["--publish"], dict(penv3, OTA_DEST="fakehost:" + dest_c6, FORCE="1"))
    dest_o = tempfile.mkdtemp(dir=dp_td)
    with open(os.path.join(dest_o, "manifest.json"), "w") as f:
        json.dump({"build": 1, "board": "GPS_LOGGER_X"}, f)
    rc_ono, _o, err_o, _w, _c = build_run(["--publish"], dict(penv3, OTA_DEST="fakehost:" + dest_o))
    check("BUILD-PUBLISH-BOARD: the manifest names its board, signed; a board's "
          "folder takes no other board's firmware, FORCE=1 or not",
          rc3 == 0 and man3.get("board") == "GPS_LOGGER_S3"
          and man3.get("hmac") == hmac3
          and rc_onc6 != 0 and "another board" in err_c6
          and rc_ono != 0 and "another board" in err_o,
          repr((rc3, err3[-200:], man3, rc_onc6, err_c6[-200:], rc_ono)))
    # an unreadable manifest is no board's: refused without FORCE
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
    with open(os.path.join(work6, "firmware", "manifest.py")) as f:
        m6 = f.read()
    fr6 = frozen_marker(work6)
    with open(os.path.join(work6, "gps_tft.py"), "rb") as f:
        tft6 = f.read()
    with open(os.path.join(work6, "gps_tft_font.py"), "rb") as f:
        tft6 += f.read()
    check("BUILD-TFT: the display's modules are frozen, and the marker carries their "
          "SHA-256 (deploy.sh compares it)",
          rc6 == 0 and 'module("gps_tft.py"' in m6
          and fr6.get("TFT") == hashlib.sha256(tft6).hexdigest(), repr((m6[-120:], fr6)))
    with open(os.path.join(work6, "gps_boot.py"), "rb") as f:
        boot6 = f.read()
    check("BUILD-BOOT: gps_boot.py is frozen and the marker carries its SHA-256 as BOOT "
          "(deploy.sh compares it: main.py's OTA and boot-retry helpers were unchecked)",
          rc6 == 0 and 'module("gps_boot.py"' in m6
          and fr6.get("BOOT") == hashlib.sha256(boot6).hexdigest(),
          repr({k: fr6.get(k) for k in ("COMMIT", "SHA", "TFT", "BOOT")}))
    # BUILD-PUBLISH-GZ: the board keeps only the gzip page: the manifest
    # names /web.html.gz alone, made with the board's small window (it
    # inflates it for a client without gzip); an older firmware (after a
    # rollback: OTA_PAGES of both pages, as on master) accepts it too
    with open(os.path.join(work, "web.html"), "rb") as f:
        pub_page = _page_min(f.read())
    pub_gz = dev_file(dest, "web-%s.html.gz" % man.get("commit")) or b""
    saved_pages = gl.OTA_PAGES
    gl.OTA_PAGES = ("/web.html", "/web.html.gz")
    try:
        old_ok = gl._ota_manifest_ok(man)
    finally:
        gl.OTA_PAGES = saved_pages
    check("BUILD-PUBLISH-GZ: the manifest names only /web.html.gz "
          "(web-<commit>.html.gz, stripped, the board's WEB_GZ_WBITS window); no "
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


# GPS-SILENT: a receiver that sends nothing was only "No data!" on the screen and a
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
        pf, _, tf = prev_after(_slurp(gl.DIAG_FILE))
    finally:
        gl.uptime_s = gl._uptime_saved
    check("UPTIME: a deliberate reset (OTA, TLS memory, low IDF, rollback) "
          "writes a status line first: the next boot sees the whole run",
          pf["up"] == 151 and pf["how"] == "status" and mem_via
          and all(src.index("_diag_final()") < src.index("machine.reset()")
                  for src in resets.values()), repr((pf, tf)))
    loop_src = inspect.getsource(gl._loop)
    check("DIAG: _loop writes a line every DIAG ms",
          "_diag, now" in loop_src and "DIAG" in loop_src)
finally:
    gl.DIAG, gl.DIAG_MAX = saved_diag


# REV29: fixes from the 2026-10-29 review (one check per finding)
def rv_nmea(body):
    c = 0
    for ch in body.encode():
        c ^= ch
    return ("$%s*%02X" % (body, c)).encode()


# GPS-2: GSA every GSA_EVERY s
check("GPS-2: GSA rate is one round per GSA_EVERY s at GPS_HZ",
      "CFGMSG,0,2,%d" % (gl.GPS_HZ * gl.GSA_EVERY) in gl._uc_msgs())

# GPS-7: a setup unanswered leaves setup_ok False with nothing extra: set up again
gl.reset()
_uart_rx[:] = []
gl.uart.silent = True
try:
    gl.gps_setup()
finally:
    gl.uart.silent = False
lost = gl.st["setup_ok"]
_uart_tx[:] = []
_clock[0] += gl.GPS_RETRY + 1000
gl.gps_check(_clock[0])
again = setup_cmds() > 0
check("GPS-7: setup not answered and no extra sentence -> sent again",
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

# NMEA-DIGITS: a time or date digit flipped past the checksum (two flipped
# bits pass the XOR) makes the sentence bad, as a bad number does. Before, the
# line "20A6-10-29T12:00:01Z" went into the track and became last_t, which no
# real time ever sorted above: no line and no anchor point until a reboot.
gl.reset()
gl.log_reset()
_nd_ld, gl.LOG_DIR = gl.LOG_DIR, os.path.join(diag_dir, "nd")
settled()
_nd_k = [0]


def _nd_sec(t, date):
    """One second of RMC + GGA, ~37 m north of the one before, then the tick."""
    _nd_k[0] += 1
    lat = "4728.%04d" % (_nd_k[0] * 20)
    _clock[0] += 1000
    gl.handle(rv_nmea("GNRMC,%s,A,%s,N,01900.0000,E,0.1,,%s,,,A,V" % (t, lat, date)))
    gl.handle(rv_nmea("GNGGA,%s,%s,N,01900.0000,E,1,12,0.9,100.0,M,40.0,M,," % (t, lat)))
    gl.log_tick()


_nd_sec("120000.00", "291026")
_nd_e0 = gl.st["err"]
_nd_sec("120001.00", "2910A6")           # the RMC bad, its GGA still a fix
_nd_sec("120002.00", "291026")
_nd_e1 = gl.st["err"]
_nd_sec("1B0003.00", "291026")           # both bad: no second started
_nd_e2 = gl.st["err"]
for _i in range(4, 12):
    _nd_sec("1200%02d.00" % _i, "291026")
_nd_lt, _nd_n, _nd_d = gl.log["last_t"], len(gl.log["batch"]), gl.st["date"]
gl.st["date"] = "20A6-10-29"            # (st set by hand: _sample alone)
_clock[0] += 1000
_nd_c = gl.log["clock"]
gl._sample(_clock[0])
check("NMEA-DIGITS: a non-digit in the time or the date makes the sentence bad, "
      "nothing of it used; the track goes on (10 lines of 11 good seconds, "
      "last_t the last one); _sample writes nothing without a readable date",
      _nd_e1 == _nd_e0 + 1 and _nd_e2 == _nd_e1 + 2 and _nd_d == "2026-10-29"
      and _nd_lt == "2026-10-29T12:00:10Z" and _nd_n == 10
      and len(gl.log["batch"]) == 10 and gl.log["clock"] == _nd_c,
      repr((_nd_e0, _nd_e1, _nd_e2, _nd_d, _nd_lt, _nd_n, len(gl.log["batch"]))))
gl.LOG_DIR = _nd_ld
gl.reset()
gl.log_reset()

# SETUP-LATE: a receiver that answers late (overrun, still booting): the setup
# ends at the first silent command, UC_ACK_WAIT later, not 10-30 s; what is
# read meanwhile goes through the parser, the banner too, and its late answers
# after that are no answer to anything
late = []
real_write, real_any = _UcUart.write, UART.any


def late_write(self, data):
    data = bytes(data)
    _uart_tx.append(data)
    late.append((_clock[0] + 1500, b"$OK*04\r\n"))
    return len(data)


def late_any(self):
    late.sort()
    while late and late[0][0] <= _clock[0]:
        _uart_rx.extend(late.pop(0)[1])
    return len(_uart_rx)


gl.reset()
_uart_rx[:] = []
_uart_tx[:] = []
late.append((_clock[0] + 100, b"FWVer R6.0.0.0Build3700\r\n"))
late.append((_clock[0] + 150, rv_nmea(
    "GNRMC,120500.00,V,,,,,,,171026,,,N") + b"\r\n"))
_UcUart.write, UART.any = late_write, late_any
t0 = _clock[0]
try:
    gl.gps_setup()
    took = _clock[0] - t0
    sl_ok = gl.st["setup_ok"]
    sl_n = setup_cmds()
    _clock[0] += 2000
    gl.read_uart()
    sl_oks = gl.st["uc_ok"]
finally:
    _UcUart.write, UART.any = real_write, real_any
    late[:] = []
check("SETUP-LATE: late answers block one UC_ACK_WAIT per command sent, the rest "
      "not sent; they are counted, no answer to anything after",
      took <= 2 * gl.UC_ACK_WAIT + 100 and sl_ok is False and sl_n == 1
      and sl_oks == 2, "ms=%d ok=%r cmds=%d oks=%d" % (took, sl_ok, sl_n, sl_oks))
check("SETUP-LATE: NMEA read during the setup is parsed (banner = restart)",
      gl.st["gps_boots"] == 1 and gl.st["ok"] >= 1 and gl.st["extra"] == 1,
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
            _uart_rx.extend(rv_nmea("GNGGA,%s,4730.000,N,01900.000,E,1,08,1.0,"
                                    "100.0,M,40.0,M,," % t) + b"\r\n")
            _uart_rx.extend(rv_nmea("GNRMC,%s,A,4730.000,N,01900.000,E,0.1,90.0,"
                                    "171026,,,A" % t) + b"\r\n")
        if gst:
            _uart_rx.extend(rv_nmea("GNGST,1206%02d.00,20,,,,3.0,2.5,6.0" % k)
                            + b"\r\n")
        gl.read_uart()
        _clock[0] += 1000
        gl.gps_check(_clock[0])
    return setup_cmds()


v_slow = verify_run(1, True)
v_good = verify_run(gl.GPS_HZ, True)
v_nogst = verify_run(gl.GPS_HZ, False)
check("SETUP-CHECK: answered but still 1 fix/s, or no GST -> set up again",
      v_slow > 0 and v_nogst > 0 and v_good == 0,
      "slow=%d nogst=%d good=%d" % (v_slow, v_nogst, v_good))

# SETUP-CHECK-RESTART: a receiver restart within GPS_VERIFY (back at 1 Hz,
# its own setup comes) is no verdict on the setup before it in the NMEA
# mode either: it was "not taken: 1 fix/s", a diag line and the
# GPS_NOT_TAKEN_SAY throttle started
gl.reset()
_uart_rx[:] = []
gl.gps_setup()
sr_pre = (gl.st["setup_ok"], gl.st["checked"])
_clock[0] += 2000
gl.handle(b"FWVer R6.0.0.0Build3700")
sr_extra = gl.st["extra"]
for k in range(12):
    _clock[0] += 1000
    t = "1200%02d.00" % k
    gl.handle(rv_nmea("GPRMC,%s,A,4730.000,N,01900.000,E,0.1,,021126,,,A" % t))
    gl.handle(rv_nmea("GPGGA,%s,4730.000,N,01900.000,E,1,08,1.0,100.0,M,"
                      "40.0,M,," % t))
    gl.handle(rv_nmea("GPGST,%s,1.0,2.0,1.0,0.0,1.5,1.2,2.0" % t))
gl._setup_check(_clock[0])
check("SETUP-CHECK-RESTART: a restart within GPS_VERIFY is no 'not taken'",
      sr_pre == (True, False) and gl.st["checked"] is False
      and gl.st["nt_said"] is None and gl.st["extra"] == sr_extra == 1,
      repr((sr_pre, gl.st["checked"], gl.st["nt_said"], gl.st["extra"])))
gl.reset()

# RATE-NAK: a rate refused ($FAIL) is an answer: it must not read as silence
# (setup_ok False, set up again and again). The rest of the setup goes on, the
# receiver stays at its rate (1 fix/s is then no "not taken" verdict), a diag
# line says so


def rate_nak_run():
    said = []
    saved_dw = gl._diag_write
    gl._diag_write = said.append
    gl.uart.bad = ("CFGNAV",)
    try:
        n = verify_run(1, True)
        state = (gl.st["setup_ok"], gl.st["rate_nak"], gl.st["checked"],
                 gl.st["extra"])
    finally:
        gl.uart.bad = ()
        gl._diag_write = saved_dw
    return n, state, [ln for ln in said if "refused" in ln]


rn1 = rate_nak_run()
check("RATE-NAK: the rate refused -> setup ok at 1 Hz, not set up again",
      rn1 == (0, (True, True, True, 0),
              ["gps refused CFGNAV,%d,%d,1000" % (1000 // gl.GPS_HZ,
                                                  1000 // gl.GPS_HZ)]),
      repr(rn1))

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
        ls = []
        if fix:
            ls.append("GNGGA,%s,4730.000,N,01900.000,E,1,08,1.0,"
                      "100.0,M,40.0,M,," % t)
            ls.append("GNRMC,%s,A,4730.000,N,01900.000,E,0.1,90.0,"
                      "171026,,,A" % t)
        else:
            ls.append("GNRMC,%s,V,,,,,,,171026,,,N" % t)
            ls.append("GNGGA,%s,,,,,0,00,99.99,,,,,," % t)
        if gst_every and k % gst_every == 0:
            ls.append("GNGST,%s,,,,,,," % t)
        for s_ in ls:
            _uart_rx.extend(rv_nmea(s_) + b"\r\n")
        gl.read_uart()
        _clock[0] += 1000 * step
        gl.gps_check(_clock[0])
        gl.read_uart()                   # (the answers of a setup it sent)


cfgs = setup_cmds


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

# NO-GST-NOFIX: without a fix (indoors) no verdict from _setup_check, GST or not:
# "no GST" was a verdict once, and indoor stretches were set up again and again.
# The UC6580 sends GST with or without a fix once it is on, so none at all for
# UC_GST_LOST is a restart whose banner was lost (_uc_watch): once, then its usual
# backoff, not a verdict
nofix_setup()
nofix_secs(0, 70, gst_every=5)
check("NO-GST-NOFIX: no fix with GST coming -> no verdict, not set up again",
      cfgs() == 0 and gl.st["checked"] is False and not gl.st["extra"],
      "cfg=%d checked=%r extra=%r" % (cfgs(), gl.st["checked"],
                                      gl.st["extra"]))
nofix_setup()
nofix_secs(0, gl.UC_GST_LOST // 1000 - 2)
ng_before = (cfgs(), gl.st["checked"], gl.st["extra"])
nofix_secs(40, 10)
check("NO-GST-NOFIX: no fix, no GST: no verdict; only UC_GST_LOST sets it up again",
      ng_before == (0, False, 0) and cfgs() > 0 and gl.st["nt_said"] is None,
      "before=%r cfg=%d" % (ng_before, cfgs()))

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

# SETUP-BACKOFF: a receiver that answers every setup but keeps sending
# the sentences it turns off: GPS_RETRY, then doubling up to a cap
gl.reset()
_uart_rx[:] = []
_uart_tx[:] = []
gl.gps_setup()
times = []
for _ in range(200):
    _clock[0] += 1500
    gl.handle(rv_nmea("GPVTG,,T,,M,0.0,N,0.0,K,A"))
    n0 = setup_cmds()
    gl.gps_check(_clock[0])
    if setup_cmds() > n0:
        times.append(_clock[0])
gaps = [(b2 - a2) // 1000 for a2, b2 in zip(times, times[1:])]
cap = getattr(gl, "GPS_IGNORED_MAX", 60000) // 1000
check("SETUP-BACKOFF: acknowledged but ignored -> retries double to a cap",
      len(gaps) >= 4 and gaps[0] <= gl.GPS_RETRY // 1000 + 2
      and gaps == sorted(gaps) and cap <= gaps[-1] <= cap + 2,
      "gaps=%s" % gaps)
gl.handle(b"FWVer R6.0.0.0Build3700")
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
        _uart_rx[:] = []
        cap_waits.append(gl.st["setup_wait"])
    except (OverflowError, MemoryError) as e_:
        cap_waits.append(repr(e_))
check("SETUP-BACKOFF-CAP: the shift is capped, the wait GPS_IGNORED_MAX",
      cap_waits == [2 * gl.GPS_RETRY, 8 * gl.GPS_RETRY]
      + [gl.GPS_IGNORED_MAX] * 3, repr(cap_waits))


# GPS-19: the rotation's first line is no status line: look in diag.1
diag_clear()
with open(gl.DIAG_OLD, "w") as f:
    f.write("boot cause 1 prev_up - mhz 80 mem 1\n"
            "up 3494 utc 20261028T125942Z nmea 1/0\n")
with open(gl.DIAG_FILE, "w") as f:
    f.write("gps setup not taken: no GST\n")
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
    f.write("gps setup not taken: no GST\n")
page = gl._web_diag()
check("DIAG-TAIL: WEB_DIAG lines of ~180 B, read in small pieces",
      len(st_line(79)) > 175 and tail == [st_line(k) for k in range(65, 80)]
      and reads and all(0 < n <= 1024 for n in reads),
      "%d lines, reads %r" % (len(tail), reads))
check("DIAG-TAIL: just after a rotation the page fills up from diag.1",
      len(page) == gl.WEB_DIAG and page[-1].startswith("gps setup not taken")
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
# at most): the page's Event log aggregates all of it
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
settled()
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

# GPS: used from the satellite list (GGA stops at 12), a banner across reads
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
ban = b"FWVer R6.0.0.0Build3700\r\n"
_uart_rx.extend(b"x" * 10 + b"\r\n")     # a cut-off line before it
gl.read_uart()
_uart_rx.extend(ban[:12])
gl.read_uart()
_uart_rx.extend(ban[12:])
gl.read_uart()
check("GPS: a start-up banner split across two reads is one restart",
      gl.st["gps_boots"] == 1, gl.st["gps_boots"])


# RXBUF: lines are read in place from one preallocated buffer (a bytes buffer
# grown per read and sliced per line copied ~1 KB at a time); a line split over
# reads, a line after it and junk still work
gl.reset()
_uart_rx[:] = []
rmcs = b"".join(rv_nmea("GNRMC,1207%02d.00,V,,,,,,,171026,,,N" % k) + b"\r\n"
                for k in range(3))
data = rmcs + rmcs
for k in range(0, len(data), 7):
    _uart_rx.extend(data[k:k + 7])
    gl.read_uart()
rx_ok = (gl.st["ok"], gl.st["err"])
_uart_rx.extend(b"\x00" * 300)               # junk: no newline in sight
gl.read_uart()
_uart_rx.extend(rmcs)
gl.read_uart()
check("RXBUF: a preallocated buffer, lines across 7-byte reads, junk dropped",
      isinstance(getattr(gl, "_rx", None), bytearray)
      and rx_ok == (6, 0) and gl.st["ok"] == 9,
      "%r ok=%d" % (rx_ok, gl.st["ok"]))



# RX-B562: nothing is framed. A garbled stream can hold 0xB5 0x62 by chance (it
# has no meaning to a UC6580): before a sentence it is what ran into it, with a
# plausible "length" behind it it holds nothing back, in a sentence that one
# sentence is bad and no other
gl.reset()
gl._rx_n = gl._rx_s = 0
_uart_rx[:] = []
_uart_rx.extend(b"\xff\xb5\x62\x01\x02\x40\x00" + rmcs)
gl.read_uart()
b62_a = (gl.st["ok"], gl.st["err"], gl._rx_n)
gl.reset()
_uart_rx.extend(b"\xb5\x62\x01\x07\xff\x03" + rmcs)
gl.read_uart()
b62_b = (gl.st["ok"], gl.st["err"], gl._rx_n)
gl.reset()
_uart_rx.extend(b"$GNRMC,120700.00,V,\xb5\x62\x01\x07\xff\x03,,,,171026,,,N*00\r\n" + rmcs)
gl.read_uart()
b62_c = (gl.st["ok"], gl.st["err"], gl._rx_n)
check("RX-B562: 0xB5 0x62 in junk is not a frame: no sentence lost or held back",
      b62_a == (3, 0, 0) and b62_b == (3, 0, 0) and b62_c == (3, 1, 0),
      repr((b62_a, b62_b, b62_c)))

# RX-TORN: however the UART cuts the stream, each line reaches handle() whole and
# in order, also across a junk drop; the buffer's bounds hold
import random as _rnd2
_tr = _rnd2.Random(7)
_tr_real = gl.handle
_tr_bad = []
_tr_max = [0]
for _ in range(120):
    _ls = [rv_nmea("GNRMC,12%04d.00,V,,,,,,,171026,,,N" % _tr.randrange(9999))
           for _k in range(_tr.randrange(1, 40))]
    _d = b"\r\n".join(_ls) + b"\r\n"
    _seen = []
    gl.handle = lambda b, a, e: _seen.append(bytes(b[a:e]))
    gl._rx_n = gl._rx_s = 0
    _uart_rx[:] = []
    _o = 0
    while _o < len(_d):
        _s = _tr.choice((1, 2, 3, 13, 63, 64, 255, 256, 257, 700))
        _uart_rx.extend(_d[_o:_o + _s])
        _o += _s
        gl.read_uart()
        _tr_max[0] = max(_tr_max[0], gl._rx_n)
    if _seen != _ls or gl._rx_n or gl._rx_s:
        _tr_bad.append((_seen[:2], _ls[:2]))
gl.handle = _tr_real
gl._rx_n = gl._rx_s = 0
gl.reset()
check("RX-TORN: lines whole and in order across any reads, the buffer within its "
      "RX_LINE + RX_READ", not _tr_bad and _tr_max[0] <= gl.RX_LINE
      and len(gl._rx) == gl.RX_LINE + gl.RX_READ, repr((_tr_bad[:1], _tr_max)))

# RXBUF-FIND: line ends are found with bytearray.find
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
              + (b"FWVer R6.0.0.0Build3700\r\n") * 2)
    _uart_rx.extend(f_data)
    gl.read_uart()
finally:
    gl._rx, gl._rxv = saved_rx
    gl._rx_n = gl._rx_s = 0
check("RXBUF-FIND: no per-byte Python loop over what the UART delivers",
      IdxCount.n < len(f_data) // 20 and gl.st["ok"] == 40
      and gl.st["gps_boots"] == 2,
      "%d indexings for %d B, ok=%d restarts=%d" % (
          IdxCount.n, len(f_data), gl.st["ok"], gl.st["gps_boots"]))

# NMEA-INPLACE: handle() reads a sentence in the buffer (read_uart's bounds,
# no copy) and decodes only the types it reads; it counts and parses as the
# copying one did (here as it was, the reference) on random sentences: bad
# checksums, odd hex, junk and blanks around, unknown types, other address
# lengths, bytes over 0x7F


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
    got = []
    for run in range(2):
        gl.reset()
        gl.st["setup_at"] = _clock[0] - 5000
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
gl._rx[:500] = bytes(range(256)) + bytes(range(244))
d_want = bytes(gl._rx[100:500])
sc = SetCount(memoryview(gl._rx))
gl._rxv, gl._rx_n = sc, 500
try:
    gl._rx_drop(100)
    dropped = (bytes(gl._rx[:400]) == d_want, gl._rx_n, sc.sets)
finally:
    gl._rxv = saved_rxv
    gl._rx_n = gl._rx_s = 0
check("RXBUF-DROP: the rest moves to the front in one slice store",
      dropped == (True, 400, 1), repr(dropped))

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
gl.reset()
gl._rx_n = gl._rx_s = 0
_uart_rx[:] = []
check("RXBUF-COMMIT: a handler's error counts the line, nothing handled twice",
      rc_got == [(None, 4, ["10:00:00", "11:00:00"], 1, 0)] * 2, repr(rc_got))


# SECONDS: 5 Hz epochs, one point a second; midnight; seconds queued behind a
# blocking call
def sec_epochs(hhmmss, date="311026", n=5, lat="4730.0000"):
    """n epochs of one second, 0.2 s apart, as the receiver sends them."""
    out = b""
    for k in range(n):
        t = "%s.%02d" % (hhmmss, k * 20)
        out += (rv_nmea("GNRMC,%s,A,%s,N,01900.0000,E,0.05,,%s,,,A,V"
                        % (t, lat, date)) + b"\r\n"
                + rv_nmea("GNGGA,%s,%s,N,01900.0000,E,1,14,1.0,100.0,M,40.0,M,,"
                          % (t, lat)) + b"\r\n")
    return out


gl.reset()
_uart_rx[:] = []
_uart_rx.extend(sec_epochs("235959", date="181026", lat="4730.0002"))
gl.read_uart()
mid0 = gl.st["seq"]
_uart_rx.extend(sec_epochs("000000", date="191026", lat="4736.0000"))
gl.read_uart()
mid1 = (gl.st["time"], gl.st["date"], gl.st["seq"], gl.st["epochs"])
_uart_rx.extend(sec_epochs("000001", date="191026", n=1))
gl.read_uart()
mid2 = (gl.st["time"], gl.st["date"], gl.st["seq"], gl.st["epochs"],
        round(gl.st["lat"], 6))
check("SECONDS: 5 epochs a second publish once; across midnight dated",
      mid0 == 0 and mid1 == ("23:59:59", "2026-10-18", 1, 5)
      and mid2 == ("00:00:00", "2026-10-19", 2, 5, 47.6),
      repr((mid0, mid1, mid2)))
q_td = tempfile.mkdtemp(prefix="secq-")
saved_dir = gl.LOG_DIR
try:
    gl.LOG_DIR = q_td
    gl.reset()
    gl.log_reset()
    settled()
    _uart_rx[:] = []
    for s_ in range(4):                   # read after a blocking POST
        _uart_rx.extend(sec_epochs("12000%d" % s_, lat="4730.%04d" % (s_ * 6)))
    gl.read_uart()
    queued = len(gl.st["q"])
    gl.log_tick()
    q_lines = list(gl.log["batch"])
finally:
    gl.LOG_DIR = saved_dir
    shutil.rmtree(q_td, ignore_errors=True)
check("SECONDS: seconds published in one read are each a point",
      queued == 3 and len(q_lines) == 3
      and q_lines[0] == "47.500000,19.000000,100.0,1.0,20261031T120000Z\n"
      and q_lines[2].startswith("47.500020,19.000000,"), repr(q_lines))
# SECONDS-THIN: a POST blocking 40 s (a weak hotspot while driving) left 40
# seconds published with no log_tick: the queue kept the newest
# SECONDS_Q, so the first 32 seconds were a gap in the track with no
# segment break. Full, it drops the second whose neighbours are closest:
# the same SECONDS_Q seconds spread over the whole block, start and end
# kept, in order, the thinned ones counted
th_td = tempfile.mkdtemp(prefix="secthin-")
saved_dir = gl.LOG_DIR
try:
    gl.LOG_DIR = th_td
    gl.reset()
    gl.log_reset()
    settled()
    _uart_rx[:] = []
    for s_ in range(41):
        _uart_rx.extend(sec_epochs("1200%02d" % s_, lat="4730.%04d" % (s_ * 12)))
    while gl.uart.any():                 # (RX_READS a call, no log_tick between)
        gl.read_uart()
    th_n = gl.st.get("q_thin")
    gl.log_tick()
    th_lines = list(gl.log["batch"])
finally:
    gl.LOG_DIR = saved_dir
    shutil.rmtree(th_td, ignore_errors=True)
th_t = [l.rstrip().rsplit(",", 1)[1] for l in th_lines]
check("SECONDS-THIN: 40 s published without log_tick keep SECONDS_Q seconds "
      "spread over all of them, not the newest only",
      len(th_lines) == gl.SECONDS_Q and th_t[0] == "20261031T120000Z"
      and th_t[-1] == "20261031T120039Z" and th_t == sorted(th_t)
      and th_n == 40 - gl.SECONDS_Q, repr((th_n, th_t)))
th_rows = dict(gl._web_rows())
check("SECONDS-THIN: status.json counts thinned seconds, lost points and track "
      "write errors", th_rows.get("q_thin") == 40 - gl.SECONDS_Q
      and "track_drop" in th_rows and "write_errs" in th_rows,
      repr({k: th_rows.get(k) for k in ("q_thin", "track_drop",
                                        "write_errs")}))

# TRK2: older tracks (a "#TRK2" file of the earlier board's NAV-PVT fixes, its
# .bad copy) still wait on a flash that came along: readers take TRK1 and TRK2
t2_td = tempfile.mkdtemp(prefix="trk2-")
saved_dir = gl.LOG_DIR
try:
    gl.LOG_DIR = t2_td
    gl.reset()
    gl.log_reset()
    path2 = os.path.join(t2_td, "20261031_120000.trk")
    with open(path2, "w") as f:
        f.write("#TRK2\n"
                "47.497912,19.040246,123.5,2,1.4,20261031T120000Z\n"
                "47.497932,19.040246,123.5,2,1.4,20261031T120001Z\n"
                "47.497952,19.040246,123.5,2,1.4,20261031T120002Z\n"
                "47.466667,19.000000,99.0,6.0,,20261031T120004Z\n"
                "47.466667,19.000000,99.0,6.0,,20261031T120005Z\n#END\n")
    path1 = os.path.join(t2_td, "20261031_130000.trk")
    with open(path1, "w") as f:
        f.write("#TRK1\n47.466667,19.000000,99.0,1.2,20261031T130000Z\n"
                "47.466778,19.000000,100.0,0.8,20261031T130002Z\n#END\n")
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

# RING-LINE: the way out rebuilt from the departure ring is the line
# _sample_line writes (single precision: mpy_unix_logger)
def ring_nmea():
    gl.reset()
    gl.log_reset()
    gl.log["still"] = sl = gl._still_state()
    sl.update(active=True, anchor=(47.5, 19.0), aref=(47.5, 19.0), n=10)
    want = []
    for k in range(gl.STILL_DEP_MAX + 7):
        tm = "10:%02d:%02d" % (k // 60, k % 60)
        gl.st.update(valid=True, date="2026-10-31", time=tm, fixtype=3,
                     lat=47.49791234567 + k * 0.0000271828,
                     lon=-0.31415 - k * 0.0000314159,
                     alt=-0.04 if k % 3 else 123.45 + k,
                     hdop=1.25 + k % 3 if k % 5 else None, hdop_at=_clock[0],
                     std=None)
        want.append(gl._sample_line(_clock[0], None, None)[1])
        gl._ring_add(sl, _clock[0], (gl.st["lat"], gl.st["lon"]), 10.0 + k)
    gl.log["batch"] = []
    gl.log["file"] = "/nonexistent/x.trb"
    gl._still_leave()
    got = list(gl.log["batch"])
    gl.log_reset()
    return want[-gl.STILL_DEP_MAX:], got


w1, g1 = ring_nmea()
check("RING-LINE: the way out is the line _sample_line writes",
      w1 == g1 and len(g1) == gl.STILL_DEP_MAX and g1[1].count(",") == 4
      and g1[1].startswith("47.498130,-0.314401,"),
      repr([(a, b) for a, b in zip(w1, g1) if a != b][:2]))
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
    return bytes(_kb_encode(ls, len(ls), None))


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
                _i7(la), _i7(lo), el / 10, acc / 10, v * 3.6, s))
        else:
            out.append("%s,%s,%.1f,%.1f,%s\n" % (
                _i7(la), _i7(lo), el / 10, acc / 10, s))
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
    hist_add(lines + ["not written\n"], 4)
    hist_add([_hl("20261031", "000002")], 1)
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
    with open(os.path.join(gl.HIST_DIR, "20261031.trk"), "w") as f:
        f.write(_hl("20261031", "120000"))   # (a day as text and .trb: one day)
    hist_add([_hl("20261101", "080000")], 1)
    kept = sorted(os.listdir(gl.HIST_DIR))
    check("HIST: a new day's file drops the days beyond HIST_DAYS (.trk "
          "and .trb alike; a day written as both counts once)",
          len(kept) == gl.HIST_DAYS + 1 and kept[-1] == "20261101.trb"
          and kept[0] == "20261006.trk" and "20261030.trb" in kept
          and "20261031.trk" in kept and "20261031.trb" in kept,
          repr(kept))
    os.remove(os.path.join(gl.HIST_DIR, "20261031.trk"))
    free_saved = gl.free
    gl.free = lambda: gl.MIN_FREE
    try:
        hist_add([_hl("20261101", "080001")], 1)
    finally:
        gl.free = free_saved
    check("HIST: nothing written while the flash is nearly full",
          _hread("20261101") == _hl("20261101", "080000"))
    # flush: a .trb's points are on the flash once, in the track file; the
    # history gets them when they are up (they were written twice, and a
    # day offline filled the flash twice as fast)
    online(_hd)
    gl.LOG_DIR = _hd
    h0 = _hread("20261101")
    gl.log["file"] = os.path.join(_hd, "20261101_090000" + gl.TRB)
    fl = [_hl("20261101", "090000"), _hl("20261101", "090001", 47.186)]
    gl.log["batch"] = list(fl)
    gl.flush()
    check("HIST: flush writes the track, not the day history",
          _hread("20261101") == h0
          and gl.log["batch"] == [] and gl.log["points"] == 2,
          repr(_hread("20261101")))
    gl.log["file"] = None                 # (closed: pending)
    _post_code[0] = 500
    pump()
    check("HIST: a failed upload adds nothing",
          _hread("20261101") == h0, repr(_hread("20261101")))
    _post_code[0] = 201
    pump()
    check("HIST: a .trb's uploaded points go to the day history, once, as its records",
          _hread("20261101") == h0 + "".join(fl)
          and not os.path.exists(os.path.join(_hd, "20261101_090000" + gl.TRB))
          and _hraw("20261101").endswith(_hkb(fl) + b"#END\n"),
          repr(_hraw("20261101")))
    pump()
    check("HIST: ... and not again", _hread("20261101") == h0 + "".join(fl))
    # a .trk from older firmware: its points went when written, not again
    h0 = _hread("20261101")
    with builtins.open(os.path.join(_hd, "20261101_090100.trk"), "wb") as f:
        f.write((HEADER + "".join(fl) + gl.FOOTER).encode())
    pump()
    check("HIST: an uploaded .trk adds nothing (older firmware's: in the history "
          "already)", _hread("20261101") == h0
          and not os.path.exists(os.path.join(_hd, "20261101_090100.trk")),
          repr(_hread("20261101")))
    os.remove(os.path.join(gl.HIST_DIR, "20261101.trb"))
    hist_add([_hl("20261101", "080000")], 1)
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
        hist_add(dl[i:i + gl.UPLOAD_BATCH],
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
    hist_add(pl[:200], 200)
    gl.HIST_REPLY = 2000
    try:
        st1, h1, b1 = _hget(b"d=%s&o=0" % day.encode())
        hist_add(pl[200:300], 100)
        t2, r2 = _hpage(day, int(h1["X-Hist-Next"]))
        e2 = r2[-1][2]
        st3, h3, b3 = _hget(b"d=%s&o=%d" % (day.encode(), e2))
        hist_add(pl[300:], 100)
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
        hist_add(ca[i:i + 50], len(ca[i:i + 50]))
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
        hist_add(cb, len(cb))
        t1_, r1_ = _hpage(day)
        tn_, _ = _hpage(day, r0_[-1][1])
        t1v = "".join(x for x in t1_.splitlines(True) if _hok(x))
        tnv = "".join(x for x in tn_.splitlines(True) if _hok(x))
        hist_add(ca[:2], 2)          # (the next append: no HIST_SEP)
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
    hist_add(el_[:60], 60)
    with builtins.open(os.path.join(gl.HIST_DIR, day + ".trb"), "r+b") as f:
        f.seek(-5, 2)
        f.write(b"\xff" * 40000)         # (no footer: the next append's
    hist_add(el_[60:], 60)           # HIST_SEP ends the block)
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
        hist_add(ll[i:i + 600], 600)
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
    hist_add([_hl("20261009", "120001")], 1)
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


saved_rv = (gl.WEB, gl.WEB_AWAY, WLAN.scan_aps, set(WLAN.ok), gl.PB_KEEPALIVE)
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

    # W-23: no .env: the scan pulses get a WLAN of their own
    gl.wifi_reset()
    gl.wifi["state"] = "no .env"
    gl._pb_w = None
    n0 = gl.wifi["pb_scans"]
    _clock[0] += gl.PB_KEEPALIVE
    gl.keepalive_tick(_clock[0])
    check("W-23: without .env the keep-alive still pulses",
          gl.wifi["pb_scans"] == n0 + 1 and gl._pb_w is not None,
          "%d" % gl.wifi["pb_scans"])

    # DHCP after a static network: always asked for once any net is static
    cfgs = []
    real_ifc = getattr(WLAN, "ifconfig", None)

    def rec_ifc(self, *a):
        if a:
            cfgs.append(a[0])
            return None
        return ("192.168.1.99", "255.255.255.0", "192.168.1.1", "1.1.1.1")
    WLAN.ifconfig = rec_ifc
    try:
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
    gl.WEB = True
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

    # WR-30: WEB on, no keep-alive (no scan sees home come back): a failed
    # round searches on with the backoff for WEB_SEARCH, then sleeps; it
    # dropped to WIFI_SLEEP (status page down 1 h) at once
    gl.PB_KEEPALIVE = 0
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
    gl.WEB, gl.PB_KEEPALIVE = False, 5000

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
    gl.WEB, gl.WEB_AWAY = True, False
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
    gl.WEB, gl.WEB_AWAY = True, True
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
    gl.WEB, gl.WEB_AWAY, gl.PB_KEEPALIVE = True, False, 0
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
    gl.WEB, gl.WEB_AWAY, gl.PB_KEEPALIVE = False, False, 5000

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
        gl.WEB, gl.PB_KEEPALIVE = True, 5000

        def r2_late_router(env_text, blank):
            r2_boot(env_text, [], [])
            gl.WEB, gl.PB_KEEPALIVE = True, 5000
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
        # keep-alive branch only: with KEEPALIVE=0 a hotspot drop (WEB_AWAY off)
        # searched WEB_SEARCH (30 min) on
        gl.WEB, gl.WEB_AWAY = True, False
        wf3 = {}
        for ka in (5000, 0):
            gl.PB_KEEPALIVE = ka
            w = rv_connect(["hot"], [("hot", -50)])
            on_hot = gl.wifi["ssid"] == "hot"
            wf3[ka] = (on_hot,) + wf_drop(w, 10 * 60)
        gl.PB_KEEPALIVE = 0
        w = rv_connect(["home"], [("home", -50)])
        wf3_home = wf_drop(w, 10 * 60)
        gl.PB_KEEPALIVE = 5000
        check("WR3-3: a hotspot drop without WEB_AWAY sleeps, keep-alive or not "
              "(a home drop with none to see it come back still searches)",
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
        gl.WEB, gl.WEB_AWAY, gl.PB_KEEPALIVE = True, False, 5000
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
        gl.WEB, gl.PB_KEEPALIVE = False, 5000
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
        check("WR3-6: an empty keep-alive scan does not count as the "
              "round's empty scan", pulsed == (1, 0)
              and order == ["home", "hot"], "%r %r" % (pulsed, order))

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
    gl.WEB, gl.WEB_AWAY, WLAN.scan_aps, WLAN.ok, gl.PB_KEEPALIVE = saved_rv
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
settled()
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
_l4 = sv_lines()
check("L-4: a queued second without a date is skipped, no crash",
      crash is None and _l4 and all(l.split(",")[4].strip()[:8] == "20261018"
                                    for l in _l4), repr((crash, _l4)))

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
    return (gl.up["state"], gl._bad_full_left())


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
      bi_res == [("fs err", False)] * 2, repr(bi_res))
check("BAD-FULL: a full .bad is BAD FULL only while that .bad is on flash",
      bf_full == ("bad full", True)
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
    ok_post = (r[0] == 201 and r[1] == b"7"
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
    ctx_retry = (ctx_fail is not None and r_again[0] == 201
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
      r301[0] == 301 and one_req)
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
                                         15)[0])
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
      r_long[0] == 429 and r_long[1] == b"9"
      and limits and all(0 < n_ <= 256 for n_ in limits),
      "%r %r" % (r_long, limits[:4]))

# HEAD-NAMES: each header in its own place, any case; HEAD-LINES: an endless
# head is read HTTP_LINES times at most, what comes later is not looked at
gl.socket, gl.select = fake_sockmod, fake_sel
sys.modules["tls"] = RvTls
try:
    rv_sock[0] = RvSock(b"HTTP/1.1 200 OK\r\nCONTENT-type: text/HTML\r\n"
                        b"Server: x\r\nretry-AFTER: 7\r\n\r\n", polls=0)
    r_names = gl._raw_post("https://d.example", b"{}", {}, 15)
    rv_sock[0] = RvSock(b"HTTP/1.1 200 OK\r\n"
                        + b"X-Pad: y\r\n" * (gl.HTTP_LINES + 20)
                        + b"Retry-After: 7\r\n\r\n", polls=0)
    r_lines = gl._raw_post("https://d.example", b"{}", {}, 15)
    n_lines = len(rv_sock[0].limits)
finally:
    gl.socket, gl.select = real_mods[0], real_mods[1]
    if real_mods[2] is None:
        sys.modules.pop("tls", None)
    tls_ctx_reset()
check("HEAD-NAMES: _raw_post gives Retry-After and Content-Type by name, any case",
      r_names == (200, b"7", b"text/HTML"), repr(r_names))
check("HEAD-LINES: a head over HTTP_LINES lines stops there, the 200 stands",
      r_lines == (200, None, None) and n_lines == gl.HTTP_LINES + 1,
      "%r %d" % (r_lines, n_lines))

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
      and run_src.index("up_reset()") < run_src.index("wifi_start("),
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
    pieces = list(gl._web_json_parts(gl._web_rows(), gl._sv_list()))
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
for n in ("20261001_100000.trk", "20261001_110000.trk",
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
banner = b"FWVer R6.0.0.0Build3700"
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
_uart_rx[:] = []
banner = b"FWVer R6.0.0.0Build3700"
for _ in range(gl.GPS_FLAP_RESTARTS + 1):
    _clock[0] += 3000
    gl.handle(banner)
gl.st["setup_at"] = _clock[0]
storm_wait = gl.st["setup_wait"]
_uart_tx[:] = []
_clock[0] += gl.GPS_STORM_QUIET + gl.GPS_RETRY + 500
gl.gps_check(_clock[0])
sent = setup_cmds() > 0
check("GPS-STORM-END: after the storm the setup comes within ~15 s, not 30-60",
      storm_wait == gl.GPS_STORM_WAIT and sent, "%r %r" % (storm_wait, sent))

# STORM-END-NOACK: the last storm setup got no ACK, but the receiver took
# the blind burst (no extra sentence): quiet after the storm, it is still
# set up within GPS_RETRY, not after the doubled GPS_STORM_WAIT
gl.reset()
for _ in range(gl.GPS_FLAP_RESTARTS + 1):
    _clock[0] += 3000
    gl.handle(banner)
gl.uart.silent = True
try:
    gl.gps_setup()
finally:
    gl.uart.silent = False
noack = (gl.st["setup_ok"], gl.st["extra"], gl.st["setup_wait"])
_uart_tx[:] = []
_clock[0] += gl.GPS_STORM_QUIET + gl.GPS_RETRY + 500
gl.gps_check(_clock[0])
sent = setup_cmds() > 0
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
                gl.handle(rv_nmea("GNRMC,120000.00,V,,,,,,,311026,,,N"))
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
page_json = _web_json()
gl.idf_heap = _idf_saved
gl.st["valid"] = False
gl.wifi.update(state="off", ssid=None, rssi=None)
gl.st["svs"] = None
page_json2 = _web_json()
# no HDOP yet (no GGA in the last STALE ms)
gl.st.update(pos_at=_clock[0], valid=True, fixtype=3, quality=1, hdop=None, lat=47.1,
             lon=18.4)
gl._web_diag = lambda: ["up 120 utc 20261031T130241Z nmea 72/0 gpsboots 0 "
                         "loops 0 sats 22/15 fix 3D_fix hdop None points 0 "
                         "sent 0 wifi connected pulses 0 mem 19280 idf "
                         "221/208 heap 62"]
page_json3 = _web_json()
gl._web_diag = _diag_saved
jd1, jd3 = json.loads(page_json), json.loads(page_json3)
check("PAGE: JSON is generated piece by piece (no list in RAM)",
      not isinstance(gl._web_json_parts(gl._web_rows(), gl._sv_list()), (list, tuple))
      and jd1["hdop"] == 0.88199996)
check("PAGE: status JSON has no field of the receiver this board does not have",
      not any(k in jd1 for k in ("hacc", "fix_src", "ubx")) and "nmea" in jd1
      and jd3["hdop"] is None, repr(sorted(jd1)))
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
    check("PAGE: telemetry shows HDOP, a dash without one, and NMEA BAD",
          "<small>HDOP</small><b>-</b>" in tm3 and "<small>ACC</small>" not in tm3
          and "<small>HDOP</small>" in tm and "<small>NMEA BAD</small>" in tm3,
          tm3[:400])
    check("PAGE: map ring from HDOP * UERE, coordinates +-that in m",
          v3.get("dom", {}).get("acc") == "36px"
          and v.get("coord", "").endswith("&plusmn;2.2 m"),
          repr((v3.get("dom", {}).get("acc"), v3.get("coord"),
                v.get("coord"))))
    # BOOT: when this run started (local time) and how long the one before
    # it lasted, until when; FIRMWARE: the build date, the commit after it
    # (RT2-8: the page takes its "now" when node runs, this test its own
    # before: across local midnight the two would disagree on "today", so the
    # run is repeated once when the day changed meanwhile)
    for _midnight in range(2):
        jb = json.loads(page_json)
        now_ = time.time()
        jb.update(uptime=10492, boot_cause=2, prev_uptime=150,
                  prev_run="2 min 30 s (last line x)",
                  prev_end=time.strftime("%Y-%m-%dT%H:%M:%SZ",
                                         time.gmtime(now_ - 10492 - 30)),
                  fw="86fb9c8", fw_build=1793548800, ota="current")
        rb_ = subprocess.run([node, os.path.join(here, "web_page_test.js"),
                              os.path.join(here, "..", "web.html")],
                             input=json.dumps(jb), capture_output=True,
                             text=True)
        if time.localtime()[:3] == time.localtime(now_)[:3]:
            break
    syb = json.loads(rb_.stdout).get("sys", "")

    def _hm(t):
        # (the page puts the month and day before a time from another day)
        lt = time.localtime(t)
        same = lt[:3] == time.localtime(now_)[:3]
        return time.strftime("%H:%M" if same else "%m-%d %H:%M", lt)

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
    check("PAGE: Systems labels every row, no UBX or fix source",
          v3.get("unlabeled") == [] and " ubx" not in sy3
          and "NAV-PVT" not in sy3, sy3[:1500])
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
            "tback": 1, "wfail": 4, "wlost": 1, "low": 1, "setup": 5,
            "env": 2, "boot": 2, "wback": 1, "stay": 1, "grx": 2,
            "other": 2}
    # the latest first; at the same last time by severity, then count
    ev_order = ["boot", "grx", "ota", "upl", "bug", "crash", "track", "full",
                "setup", "env", "otax", "reboot", "tback", "wlost", "low",
                "wback", "stay", "other", "gsil", "gloop", "wfail"]
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
          and evg.count('<div class="ev-grp">') == 21
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
    # the important ones in their own kind (the display's backlight too),
    # Other only for the clock and the heap notes. Every format in the source must have a sample here
    ev_samples = [
        ("gps reset loop 5 restarts in 6 s", "gloop"),
        ("gps reset loop 3 more windows", "gloop"),
        ("gps refused CFGMSG,0,7,5", "setup"),
        ("gps systems h135155", "grx"),
        ("gps refused CFGSYS,h135155", "setup"),
        ("gps setup lost: no GST", "setup"),
        ("gps setup not taken: no GST (2 more)", "setup"),
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
        ("clock ntp 2026-11-02T11:38:31Z", "other"),
        ("tft light 0", "disp"),
        ("tft light 100", "disp"),
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
        ("ota note unreadable, dropped (running 2e8bc6e)", "otax"),
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
    for m_ in re.finditer(r'(_diag_write|_ota_say|_say|_loop_diag|'
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
          "upload errors, OTA, GPS silent / reset loop / setup, WiFi, "
          "track and clock, reboots, boots, receiver, display backlight; "
          "Other only the clock set and heap notes" % len(ev_fmts),
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
    check("PAGE-DOM-ERR: an error of the board's own (text/plain: 503, 404; no "
          "JSON) is a board error with its status, as a 500 is, not OFFLINE",
          dom.get("err503") == ["off", "BOARD ERROR &middot; HTTP 503 busy"]
          and dom.get("err404") == ["off", "BOARD ERROR &middot; HTTP 404 "
                                           "not found"]
          and dom.get("err500") == ["off", "BOARD ERROR &middot; "
                                           "TypeError(&#39;x&#39;)"],
          repr((dom.get("err503"), dom.get("err404"), dom.get("err500"))))
    check("PAGE-DOM-ERR-GATEWAY (RT2-3): a gateway's or a captive portal's "
          "HTML page (502, 403) or an empty 204 is not the board's: OFFLINE, "
          "not BOARD ERROR",
          all(dom.get(k) == ["off", "OFFLINE &middot; 0 S AGO"]
              for k in ("err502", "err403", "err204")),
          repr([dom.get(k) for k in ("err502", "err403", "err204")]))
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
    check("PAGE-PLACE-NAME: a station by its name, else city and street (no number, no shop's "
          "name), a road's own name, the district; a name once; nothing named: empty",
          ft.get("placeOf") == ["Nagyváros-Kelet", "Kisváros, Fő utca",
                                "Nagyváros, Hosszú út", "Kisváros, Fő utca",
                                "Kisváros, Szélső", "Nagyváros", "", ""],
          repr(ft.get("placeOf")))
    _pl = ft.get("placeLine") or ""
    check("PAGE-PLACE-LINE: a known place over its line, escaped, the row taller; an unknown one "
          "leaves the row exactly as without places",
          '<div class="tr go pl" onclick="hlTrip(0)"' in _pl and "<strong>A &lt;b&gt;</strong><br><b>" in _pl
          and '<div class="tr st pl">' in _pl and "<strong>B</strong><br>stop 2 h 40 min" in _pl
          and '<div class="tr go on" onclick="hlTrip(1)"' in _pl
          and '<div class="tr end pl"><time>11:10</time><i></i><span><strong>C</strong></span>' in _pl
          and ft.get("placeSame") is True, _pl[:400])
    _pq = ft.get("placeQ") or {}
    check("PAGE-PLACE-ASK: each place once and one at a time, a railway station within 150 m "
          "first, then what is there; a trip still going not its end; kept, and known ones ask "
          "nothing; no answer or an HTTP error waits and tries again",
          _pq.get("first") == [["", ""], ["", ""]]
          and _pq.get("queued") == ["47.000,18.000", "47.100,18.000"]
          and _pq.get("oneAtATime") == 1
          and "lat=47.00001&lon=18.00000&limit=1&radius=0.15&osm_tag=railway:station" in _pq.get("url0", "")
          and _pq.get("url1", "").endswith("lat=47.00001&lon=18.00000&limit=1")
          and "lat=47.10000&lon=18.00000&limit=1&radius=" in _pq.get("url2", "")
          and _pq.get("after") == [3, 0, "C, S", "St"]
          and _pq.get("known") == [["C, S", "St"], ["St", ""]] and _pq.get("again") == 3
          and _pq.get("failed") == [False, 1, True] and _pq.get("waits") == 4
          and _pq.get("http500") == [1, True] and _pq.get("none") == ["", 0], repr(_pq))
    _pr = ft.get("placeRetry") or {}
    check("PAGE-PLACE-RETRY: a place asked in vain is asked again after PLACE_WAIT by "
          "the poll itself, the track unchanged (a finished day, 1 trip: 2 places "
          "queued); its name then shows and the next place is asked",
          _pr.get("before") == [20, 1, 1, 2] and _pr.get("soon") == 1
          and _pr.get("again") == 2 and _pr.get("after") == [3, True, 1], repr(_pr))
    _ph = ft.get("placeHol") or {}
    check("PAGE-PLACE-HOL (RT2-1): a place the geocoder never answers (429) goes "
          "to the back of the queue after 3 failed tries, so the next one is "
          "asked and named within 3 min 15 s; one request at a time, the "
          "failing one still tried about once a minute (12 min: 8-14 asks)",
          _ph.get("queued") == ["47.100,18.400", "47.119,18.400"]
          and _ph.get("namedAt") is not None and _ph["namedAt"] <= 195
          and _ph.get("maxFlight") == 1 and 8 <= _ph.get("headAsks", 0) <= 14
          and _ph.get("left") == ["47.100,18.400"], repr(_ph))
    check("PAGE-PLACE-EVICT (RT2-9): a full place cache (PLACE_MAX) drops its "
          "oldest name, not all of them",
          (ft.get("placeEvict") or {}).get("keys") == ["c", "d", "e"]
          and (ft.get("placeEvict") or {}).get("queue") == 0,
          repr(ft.get("placeEvict")))
    check("PAGE-NUM-ESC (RT2-2): the counters (left, sent, upload_err, "
          "bad_files, points, files, pulses, gps_loops, gps_boots) are "
          "escaped whatever type the JSON gave; the digits still grouped",
          (ft.get("numEsc") or [None])[0] == []
          and (ft.get("numEsc") or [0, 0])[1] is True
          and (ft.get("numEsc") or [0] * 5)[2:] == ["1\u202f234\u202f567", "-",
                                                    "&lt;b&gt;"],
          repr(ft.get("numEsc")))
    check("PAGE-OTA-MISSING (RT2-9): an fw without an ota in the JSON shows a "
          "dash, not \"ota undefined\"",
          ft.get("otaMissing") == [True, True], repr(ft.get("otaMissing")))
    check("PAGE-EV-DISP (RT2-9): the display's \"tft light N\" diag lines are "
          "their own quiet Display backlight group (count, last values), "
          "not Other",
          ft.get("dispKind") == [[["disp", "Display backlight", 3,
                                   "30 %, 100 %"]], True, True],
          repr(ft.get("dispKind")))
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
          and v.get("tile", {}).get("y") == 46048
          and abs(v.get("tile", {}).get("mpp", 0) - 0.8130) < 0.0001, repr(v.get("tile")))
with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                       "web.html")) as f:
    page_src = f.read()
# PAGE-DEAD: no rule without an element, no second copy of the maths. A rule
# for a class neither the markup nor the script names is dead; q-* and t-* are
# put together in the script
_pd_css = page_src[page_src.index("<style>"):page_src.index("</style>")]
_pd_rest = page_src[page_src.index("</style>"):]
_pd_sel = re.sub(r"\{[^}]*\}", "{}", re.sub(r"/\*.*?\*/", "", _pd_css, flags=re.S))
_pd_dead = sorted(c for c in set(re.findall(r"\.([a-zA-Z][\w-]*)", _pd_sel))
                  if not re.match(r"[qt]-", c)
                  and not re.search(r"(?<![\w-])" + re.escape(c) + r"(?![\w-])", _pd_rest))
_pd_gone = [w_ for w_ in (".grid", 'class="grid"', ".r .u", ".sx-big.bad", ".gt-dg svg", "var SHOWN",
                          "var OTHER", "function tile(", "Math.cos(pts[i][1]")
            if w_ in page_src]
_pd_one = {w_: page_src.count(w_) for w_ in (
    ".gt-ax{", "Math.log(Math.tan(r)", "<path d=\"M12 7v5l3 2\"/>", "Math.sqrt(Math.pow(",
    "String(d.position || \"\").split", "d.hdop * UERE", "<em>min</em>\" : Math.floor(")}
_pd_null = re.findall(r"(\w[\w.]*) [!=]== null (?:\|\||&&) \1 [!=]== undefined", page_src)
check("PAGE-DEAD: every CSS class is in the markup or the script; the dead rules and the "
      "test-only lists gone; one copy each of the Mercator, metres, the position and the "
      "accuracy; == null, not both tests; the classes the tests read still drawn",
      not _pd_dead and not _pd_gone and set(_pd_one.values()) == {1} and not _pd_null
      and all(w_ in page_src for w_ in ('"now top"', '"st" : "end"', '"fa"')),
      repr((_pd_dead, _pd_gone, _pd_one, _pd_null)))
# WEB-MIN: tools/web_min.py strips the page at build time (a // in a string or
# a regex is no comment: the page has both); web.html keeps its comments
_wm_js = (
    ('var u = "https://atlas.crusand.org/x"; // a url\n', 'var u = "https://atlas.crusand.org/x";'),
    ("var u = 'a//b', v = \"/*no*/\", w = 'it\\'s // no';", "var u = 'a//b', v = \"/*no*/\", w = 'it\\'s // no';"),
    ('x = s.replace(/\\/\\/.*$/, ""); // gone', 'x = s.replace(/\\/\\/.*$/, "");'),
    ('if (/[/]\\/\\/"/.test(s)) y(); /* c */ z();', 'if (/[/]\\/\\/"/.test(s)) y(); z();'),
    ("a = b / c; // d / e\n", "a = b / c;"),
    ('if (x) /\\/\\/"/.test(y); // z', 'if (x) /\\/\\/"/.test(y);'),
    ("while (x) /'/.test(y)", "while (x) /'/.test(y)"),
    ("n = (a + b) / 2 / c; // x", "n = (a + b) / 2 / c;"),
    ('i++ / 2 // "\n', "i++ / 2"),
    ("return /'//* c */;", "return /'/ ;"),
    ("a = [1] / 2; b = x / 3; c = 4 / 5 // q", "a = [1] / 2; b = x / 3; c = 4 / 5"),
    ('f(/a/g, /b\\/c/i, !/d/.test(e), x ? /"/ : /\'/); // r', 'f(/a/g, /b\\/c/i, !/d/.test(e), x ? /"/ : /\'/);'),
    ("function f() {\n    // c\n\n    return 1;   // d\n}\n", "function f() {\nreturn 1;\n}"),
    ("var a = 1 /* x */ + 2;", "var a = 1 + 2;"),
    ("a /* one\ntwo */ b", "a\nb"),
    ('s = "a\\\n   b"; // c', 's = "a\\\n   b";'),
    ("// it's\nx = 1;", "x = 1;"),
    ("x  =  a   +   b  ;", "x = a + b ;"),
    ("a = b\n\n\n  c = d", "a = b\nc = d"),
)
_wm_got = [(web_min.minify_js(a_), b_) for a_, b_ in _wm_js] if web_min else []
check("WEB-MIN-JS: comments and indentation out; a // or /* in a string or a regex "
      "stays, a division is no regex, the lines and the spaces around operators stay",
      web_min and all(a_ == b_ for a_, b_ in _wm_got), repr([g_ for g_ in _wm_got if g_[0] != g_[1]]))
_wm_cases = web_min and (
    web_min.minify_css('a{content:"/* no */"} /* yes */\n   b{c:url(x/*y)}\n\n') == 'a{content:"/* no */"}\nb{c:url(x/*y)}',
    web_min.minify_css("a/**/b{}") == "a b{}",
    web_min.minify('<a href="https://x//y">t</a><!-- c -->\n   <b>  x</b>\n\n<style>\n/*c*/ a{b:c}\n</style>'
                   '<script>\n// s\nf("//");\n</script>\n')
    == '<a href="https://x//y">t</a>\n<b>  x</b>\n<style>\na{b:c}\n</style><script>\nf("//");\n</script>\n')
check("WEB-MIN-CSS: comments out, a /* in a CSS string or a url() stays; markup: comments "
      "and indentation out, a // in an attribute stays", _wm_cases and all(_wm_cases), repr(_wm_cases))
_wm_bad = []
if web_min:
    for fn_, src_ in ((web_min.minify_js, '"cut\n"'), (web_min.minify_js, "a = /cut\n/"),
                      (web_min.minify_js, "/* cut"), (web_min.minify_js, "a = `b`"),
                      (web_min.minify_css, "a{b:'cut\n}"), (web_min.minify_css, "/* cut")):
        try:
            _wm_bad.append(fn_(src_))
        except web_min.MinifyError:
            pass
_wm_td = tempfile.mkdtemp(prefix="webmin-")
_wm_broken = os.path.join(_wm_td, "broken.html")
with open(_wm_broken, "w") as f:
    f.write('<script>\nvar a = "cut;\n</script>\n')
_wm_tool = os.path.join(ROOT, "tools", "web_min.py")
_wm_r = subprocess.run([sys.executable, _wm_tool, _wm_broken, os.path.join(_wm_td, "o.gz")],
                       capture_output=True, text=True)
check("WEB-MIN-FAIL: what it cannot read (a string, regex or comment cut short, a template "
      "literal) stops the build, nothing written",
      web_min and not _wm_bad and _wm_r.returncode == 1 and "cut short" in _wm_r.stderr
      and not os.path.exists(os.path.join(_wm_td, "o.gz")), repr((_wm_bad, _wm_r.stderr[-200:])))
_wm_page = web_min.minify(page_src) if web_min else page_src
_wm_css = lambda t_: re.sub(r"\s+", " ", re.sub(r"/\*.*?\*/", " ", t_[t_.index("<style>"):t_.index("</style>")],
                                                flags=re.S)).replace(" }", "}").strip()
_wm_html = lambda t_: re.sub(r"\s+", " ", re.sub(r"<!--.*?-->", " ", t_[:t_.index("<style>")]
                                                 + t_[t_.index("</style>"):t_.index("<script>")], flags=re.S))
_wm_gz = os.path.join(_wm_td, "page.gz")
_wm_r = subprocess.run([sys.executable, _wm_tool, os.path.join(ROOT, "web.html"), _wm_gz],
                       capture_output=True, text=True)
_wm_data = b""
if os.path.exists(_wm_gz):
    with open(_wm_gz, "rb") as f:
        _wm_data = f.read()
_wm_w = getattr(gl, "WEB_GZ_WBITS", 12)
check("WEB-MIN-PAGE: the page stripped and gzipped with the board's window, no name or "
      "mtime, not over 29 KB (35.7 before); the CSS and the markup only lose comments and "
      "white space; stripping again changes nothing; web.html itself keeps its comments",
      web_min and _wm_r.returncode == 0
      and _gunzip_w(_wm_data, _wm_w) == _wm_page.encode("utf-8") and _wm_data[4:8] == b"\0\0\0\0"
      and web_min.wbits() == _wm_w and 0 < len(_wm_data) <= 29000
      and web_min.minify(_wm_page) == _wm_page and "<!--" not in _wm_page
      and "/*" not in _wm_page[_wm_page.index("<style>"):_wm_page.index("</style>")]
      and _wm_css(_wm_page) == _wm_css(page_src) and _wm_html(_wm_page) == _wm_html(page_src)
      and "<!--" in page_src and "// " in page_src,
      repr((_wm_r.returncode, _wm_r.stderr[-200:], len(_wm_data), len(_wm_page))))
if node is not None:
    _wm_min = os.path.join(_wm_td, "web.html")
    with open(_wm_min, "w", encoding="utf-8") as f:
        f.write(_wm_page)
    _wm_here = os.path.join(ROOT, "tests", "web_page_test.js")

    def _wm_view(path, js):
        return subprocess.run([node, _wm_here, path], input=js, capture_output=True,
                              text=True).stdout

    _wm_same = []
    for js_ in (page_json, page_json2, page_json3):
        # (a minute turning over between the two runs changes a printed time)
        for _ in range(3):
            a_ = _wm_view(os.path.join(ROOT, "web.html"), js_)
            if a_ and a_ == _wm_view(_wm_min, js_):
                _wm_same.append(True)
                break
        else:
            _wm_same.append(False)
    check("WEB-MIN-RUN: tests/web_page_test.js on the stripped page prints what it prints "
          "on web.html, all it runs (view, the DOM stub, the map, the track, the event "
          "log) on three status answers", web_min and _wm_same == [True] * 3, repr(_wm_same))
    # the same program: acorn (inside node) on both, the positions left out
    _wm_ast = (
        "const a = require('internal/deps/acorn/acorn/dist/acorn');"
        "const fs = require('fs');"
        "const js = (p) => { const h = fs.readFileSync(p, 'utf8');"
        " return h.slice(h.indexOf('<script>') + 8, h.lastIndexOf('</script>')); };"
        "const c = [];"
        "const t = (s, o) => JSON.stringify(a.parse(s, Object.assign({ecmaVersion: 'latest'}, o)),"
        " (k, v) => k === 'start' || k === 'end' ? undefined : typeof v === 'bigint' ? String(v) : v);"
        "const x = t(js(process.argv[1])), y = t(js(process.argv[2]), {onComment: c});"
        "console.log(JSON.stringify({same: x === y, comments: c.length, n: x.length}));")
    _wm_r = subprocess.run([node, "--expose-internals", "-e", _wm_ast, os.path.join(ROOT, "web.html"), _wm_min],
                           capture_output=True, text=True)
    if "Cannot find module" in _wm_r.stderr or "bad option" in _wm_r.stderr:
        print("SKIP  WEB-MIN-AST: this node has no acorn to reach")
    else:
        try:
            _wm_j = json.loads(_wm_r.stdout)
        except ValueError:
            _wm_j = {"error": _wm_r.stderr[-300:]}
        check("WEB-MIN-AST: the stripped script parses to the same program as the source "
              "(every string and regex with its raw text), no comment left",
              web_min and _wm_j.get("same") is True and _wm_j.get("comments") == 0, repr(_wm_j))
shutil.rmtree(_wm_td, True)
check("PAGE-STATIC-GONE: no Static test panel, code, CSS or browser "
      "storage on the page",
      not any(w_ in page_src for w_ in (
          "Static test", "statView", "statAdd", "sstat", "scat", "STAT_",
          "gps-static", "localStorage", "CEP50")), "")
# (the map: OSM-carto tiles through the homelab Atlas proxy, as Dawarich
# uses: one cache, Geoapify's key stays on CT 146; OSM's own tile servers
# ask apps not to use them for this)
# PAGE-FONTS (RT2-9): the Google Fonts sheet must not hold the first paint back
# when the phone has no internet (a render-blocking <link rel=stylesheet>
# waits for its fetch to fail): media=print, switched to all by its own load;
# <noscript> keeps it for a browser without scripts
_fonts_links = re.findall(r"<link[^>]*fonts\.googleapis\.com[^>]*>", page_src)
_fonts_min = web_min.minify(page_src) if web_min else page_src
check("PAGE-FONTS (RT2-9): the font sheet loads as media=print and is switched "
      "on by its onload (no render blocking offline), a <noscript> link keeps "
      "it for no-script browsers",
      len(_fonts_links) == 2
      and 'media="print"' in _fonts_links[0]
      and "onload=\"this.media='all'\"" in _fonts_links[0]
      and "media=" not in _fonts_links[1]
      and "<noscript>" + _fonts_links[1] + "</noscript>" in page_src
      and "media=\"print\"" in _fonts_min and "<noscript>" in _fonts_min
      , repr(_fonts_links))
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
# over each other. Runs wherever google-chrome or chromium is on the PATH, else
# SKIP; the CI workflow installs neither, so there it runs only if the runner
# image has one.
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
              "pulses 3 mem 27000 idf 157/120 heap 124",
              "boot cause 1 prev_up 700 mhz 80",
              "upload err OSError 113 ECONNABORTED while posting the batch"],
        satellites=[{"name": "G12", "system": "GPS", "svid": 12, "cno": 38,
                     "elev": 45, "azim": 120, "used": True,
                     "unhealthy": False}],
        receiver="UC6580I G1B1L1E1, R6.0.0.0Build3700", rx_bytes=123456, rx_age_s=0,
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

# WEB-LINK: the page's GPS link - bytes read, seconds since the last whole
# NMEA sentence (a bad checksum is not one), None before any
def _nm(body):
    c = 0
    for ch in body.encode():
        c ^= ch
    return ("$%s*%02X\r\n" % (body, c)).encode()
gl.reset()
j0 = json.loads(_web_json())
_junk = b"\x00\xff garbage\r\n$GPRMC,1,2*00\r\n"
_uart_rx.extend(_junk)
gl.read_uart()
j1 = json.loads(_web_json())
junk_b, junk_good = gl.st["rx_b"], gl.st["good"]
_clock[0] += 4000
s1 = _nm("GPRMC,120500.00,V,,,,,,,171026,,,N")
_uart_rx.extend(s1)
gl.read_uart()
good_at = gl.st["good"]
_clock[0] += 7000
s2 = _nm("GNRMC,120501.00,V,,,,,,,171026,,,N")
_uart_rx.extend(s2)
gl.read_uart()
ubx_at = gl.st["good"]
_clock[0] += 12000
_bad_ck = b"$GNRMC,120502.00,V,,,,,,,171026,,,N*00\r\n"
_uart_rx.extend(_bad_ck)                  # bytes came, no whole sentence
gl.read_uart()
j2 = json.loads(_web_json())
check("WEB-LINK: rx_bytes counts every byte, good_age_s only whole frames",
      j0["rx_bytes"] == 0 and j0["rx_age_s"] is None
      and j0["good_age_s"] is None
      and junk_b == len(_junk) and junk_good is None and j1["good_age_s"] is None
      and j1["rx_age_s"] == 0 and good_at is not None
      and ubx_at == good_at + 7000
      and j2["rx_bytes"] == len(_junk) + len(s1) + len(s2) + len(_bad_ck)
      and j2["good_age_s"] == 12 and j2["rx_age_s"] == 0,
      repr((j0.get("rx_bytes"), junk_b, junk_good, j1.get("good_age_s"),
            good_at, ubx_at, j2.get("rx_bytes"), j2.get("good_age_s"))))

# WEB-POS6: the position at the track line's precision for the static test
gl.st.update(pos_at=_clock[0], valid=True, lat=47.1853612, lon=18.4273049)
p_nm = json.loads(_web_json())
gl.st["valid"] = False
p_none = json.loads(_web_json())
check("WEB-POS6: pos6 %.6f, None w/o fix",
      p_nm["pos6"] == "47.185361, 18.427305"
      and p_nm["position"] == "47.18536, 18.42730"
      and p_none["pos6"] is None,
      repr((p_nm.get("pos6"), p_none.get("pos6"))))
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
days3 = json.loads(_web_json())["days"]
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
ua0 = json.loads(_web_json())["upload_age_s"]
_post_code[0] = 422
pump()
ua1 = gl.up["ok_at"]
online(_ua_dir)
make_trk(os.path.join(_ua_dir, "20261031_130000.trk"), 3)
pump()
_clock[0] += 61000
ua2 = json.loads(_web_json())["upload_age_s"]
shutil.rmtree(_ua_dir, ignore_errors=True)
check("WEB-UPLOAD-AGE: upload_age_s None until a 2xx, then its age",
      ua0 is None and ua1 is None and ua2 == 61 and gl.up["sent"] == 3,
      repr((ua0, ua1, ua2, gl.up.get("sent"))))


# UART-PINS: the UC6580 on UART1 GPIO34 (TX) / GPIO33 (RX) at its 115200, a buffer
# that rides out HTTP calls
_uart_kw = []
_UART0 = sys.modules["machine"].UART
class _UARTSpy(_UART0):
    def __init__(self, *a, **k):
        _uart_kw.append((a, k))
        _UART0.__init__(self, *a, **k)
sys.modules["machine"].UART = _UARTSpy
try:
    gl = _reload_logger()
finally:
    sys.modules["machine"].UART = _UART0
    gl = _reload_logger()
check("UART-PINS: UART1 tx=34 rx=33 at 115200, rxbuf 8 KB",
      _uart_kw[-1] == ((1,), {"baudrate": 115200, "tx": 34, "rx": 33, "rxbuf": 8192}),
      repr(_uart_kw[-1]))

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
r1a = json.loads(_web_json())
_clock[0] += gl.NO_DATA + 1000
r1b = json.loads(_web_json())
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
    gl._pb_pulse_radio(_ScanW("mem"))
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

# REV-13: a receiver restart read during an answered setup keeps the wait
# _gps_restarted set (a storm's GPS_STORM_WAIT), not GPS_RETRY again
gl.reset()
_uart_rx[:] = []
gl.st["storm_at"] = _clock[0]
_real_uart = gl.uart


class _UcRestart(_UcUart):
    said = False

    def write(self, data):
        n = super().write(data)
        if not self.said:                 # the banner with the first answer
            self.said = True
            _uart_rx.extend(b"FWVer R6.0.0.0Build3700\r\n")
        return n


gl.uart = _UcRestart()
try:
    gl.gps_setup()
    b13 = gl.st["gps_boots"]
finally:
    gl.uart = _real_uart
    _uart_rx[:] = []
w13 = gl.st["setup_wait"]
gl.reset()
check("REV-13: a restart during the setup keeps the storm wait",
      b13 == 1 and w13 == gl.GPS_STORM_WAIT, repr((b13, w13)))

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


# REV-16: an append after one cut short (half a line, no newline) starts
# on a new line: no ghost point glued from the two
_r16 = tempfile.mkdtemp()
_hd16 = gl.HIST_DIR
gl.HIST_DIR = _r16
with open(os.path.join(_r16, "20261101.trk"), "w") as f:
    f.write("47.100000,18.400000,1,2,1,20261101T100000Z\n47.4979")
hist_add(["47.200000,18.500000,1,2,1,20261101T100001Z\n"], 1)
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
         "board": gl.OTA_BOARD,
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
          "board": mm["board"], "files": [], "hmac": mm["hmac"]}
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

    # GD-OTA-SHA: the page's own sha256 (same size: only the hash tells), and
    # the slot read back from flash (the stream's hash matches a bad write)
    _ota_reset()
    gl.FW_BUILD, gl.FW_COMMIT = 100, "abc0100"
    _serve(_manifest(200))
    _files["http://ota.lan/gps/web.html.gz"] = b"x" * len(_page)
    try:
        gl._ota_check()
    except ValueError:
        pass
    gd_page = (list(_ota_log), os.path.exists(os.path.join(_ota_dir, "web.html.gz.new")),
               os.path.exists(os.path.join(_ota_dir, "web.html.gz.tmp")))
    check("GD-OTA-PAGE: a page of the right size failing its sha256 is not staged, "
          "nothing boots", gd_page == ([], False, False), repr(gd_page))
    _ota_reset()
    _serve(_manifest(200))
    gd_rb = _slots[1].readblocks
    def _gd_bad_read(blk, buf):
        gd_rb(blk, buf)
        buf[0] ^= 1                      # what the flash holds is not what went in
    _slots[1].readblocks = _gd_bad_read
    try:
        gl._ota_check()
    finally:
        del _slots[1].readblocks
    check("GD-OTA-READBACK: a slot whose read-back differs from its sha256 is a bad "
          "image, nothing boots",
          _ota_log == [] and gl.ota["state"] == "bad image",
          repr((_ota_log, gl.ota["state"])))

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
        # GD-OTA-LIMIT: no Content-Length: the limit holds while reading
        _gs[0] = _GetSock(b"HTTP/1.0 200 OK\r\n\r\n" + b"y" * 9000)
        out = bytearray()
        try:
            gd_lim = gl._raw_get("http://h/x", out.extend, 5000)
        except ValueError as e:
            gd_lim = str(e)
        check("GD-OTA-LIMIT: a body with no length stops at the limit, not at the "
              "flash's end", gd_lim == "ota body over 5000" and len(out) <= 5000,
              repr((gd_lim, len(out))))
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
    jd = json.loads(_web_json())
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
        src = _slurp(mp)
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
        # (a status line runs to ~220 B after weeks up: the cap is above it,
        # or its last field was cut off the page's donut)
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
        exec(compile(_slurp(mp), mp, "exec"), ns)
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
        exec(compile(_slurp(mp), mp, "exec"), ns)
        saved = (gb.OTA_PENDING, gb.OTA_MARK, ns["time"].sleep,
                 sys.modules.get("gps_frozen"))
        gb.OTA_PENDING, gb.OTA_MARK = gl.OTA_PENDING, gl.OTA_MARK
        ns["time"].sleep = lambda s: None
        fz = sys.modules["gps_frozen"] = types.ModuleType("gps_frozen")
        fz.BUILD = 200                   # (the running firmware: the note's)
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
            gb.OTA_PENDING, gb.OTA_MARK, ns["time"].sleep = saved[:3]
            if saved[3] is None:
                del sys.modules["gps_frozen"]
            else:
                sys.modules["gps_frozen"] = saved[3]
            RTC.mem = b""
        return out
    mc = _guarded(_t_main_crash)
    check("MAIN-CRASH: a logger failing in main.py while an OTA is pending "
          "marks the build's run as a crash", mc == ["crash", None], repr(mc))
    # OTA-DOC: the boot-bypass pin is GPIO7, not GPIO25
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
    gl._pb_pulse_radio(_EmptyW())
    first_pulse = list(stays)
    gl._pb_pulse_radio(_EmptyW())
finally:
    gl._stay_scan = _ss
check("R2-7: one empty scan is a failed one, two in a row a place without "
      "APs (same rule for the keep-alive scans)",
      e7 == [None, []] and first_pulse == [0] and stays == [0, 0],
      repr((e7, first_pulse, stays)))
gl.wifi_reset()



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
    w2_errs = json.loads(_web_json()).get("web_errs")
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
    gl.up["next_at"], gl.up["backoff"] = None, gl.BACKOFF_START
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
        raise ValueError("health bug")   # (a MemoryError is no crash: RN-1)
    gl._tick_bug.clear()
    gl.ota["pend"] = 123
    try:
        gl._guarded("ota", _gi_end)
        gi_pend = "swallowed"
    except ValueError:
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
    gl.up["next_at"], gl.up["backoff"] = None, gl.BACKOFF_START
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
    wj_text = "".join(gl._web_json_parts(gl._web_rows(), gl._sv_list()))
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
    """n text lines: a 1 Hz drive (fmt 2: older NAV-PVT tracks, ints, acc and
    speed; fmt 1: as _line writes them, NMEA floats and hdop)."""
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
            c = _i7(li) + "," + _i7(lo)
            out.append("%s,%s,%d,%s,%s\n" % (c, e, max(1, int(hd * 2)),
                                             "%.1f" % spd, tc))
        else:
            c = "%.6f,%.6f" % (li / 1e7, lo / 1e7)
            out.append("%s,%s,%s,%s\n" % (c, e, "%.1f" % hd, tc))
    return out


def _tb_write(path, lines, per=60, fmt=None):
    """lines through flush() into path, per lines a flush (as the logger
    writes a drive: FLUSH every 60 s). A .trk: the text track of older firmware
    with the same lines (its writer is gone)."""
    gl.log_reset()
    if path.endswith(".trk"):
        with builtins.open(path, "wb") as f:
            f.write(("#TRK2\n" if lines[0].count(",") == 5 else HEADER).encode()
                    + "".join(lines).encode() + gl.FOOTER.encode())
        return
    gl.log["file"] = path
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
        f.write(gl.HEADERB.encode() + _kb_encode(far, len(far), None)
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
          and not bad.endswith(gl.TRB)
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
    ub = bytearray(gl.HEADERB.encode() + _kb_encode(ul, len(ul), None)
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
        fz[fmt] = (gl.HEADERB.encode() + _kb_encode(fl, len(fl), None)
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

    # the open file's name
    gl.st["date"], gl.st["time"] = "2026-11-01", "13:00:00"
    gl.start_file()
    nm1 = gl.log["file"]
    gl.log_reset()
    check("TRKB: new files are .trb", nm1.endswith("20261101_130000.trb"), repr(nm1))
finally:
    gl.LOG_DIR, gl.HIST_DIR, gl.UPLOAD_BATCH = _tb_saved
    shutil.rmtree(_tb, ignore_errors=True)
    gl.log_reset()
    gl.up_reset()


# --- The Heltec Wireless Tracker (UC6580) -----------------------------------
# The sentences here are the board's, measured 2026-11-04 (receiver UC6580I
# R6.0.0.0Build3700, indoors: no signal, so the fix sentences are made up).
def _heltec_logger():
    spec = importlib.util.spec_from_file_location(
        "gps_logger_heltec", os.path.join(ROOT, "gps_logger.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    m.DIAG_FILE = os.path.join(diag_dir, "diag-h.log")
    m.DIAG_OLD = os.path.join(diag_dir, "diag-h.1")
    m.LOWIDF_MARK = os.path.join(diag_dir, "lowidf-h")
    m.HIST_DIR = os.path.join(diag_dir, "hist-h")
    m.HOME_APS_FILE = None
    m.WEB = False
    return m


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
      "backlight (GPIO21) off, before any UART use",
      (hl.GPS_UART, hl.GPS_TX, hl.GPS_RX, hl.BAUD) == (1, 34, 33, 115200)
      and [p[:2] for p in _pin_levels if p[0] in (3, 21)] == [(3, 1), (21, 0)],
      repr((hl.GPS_TX, hl.GPS_RX, hl.BAUD, _pin_levels[:4])))
with open(os.path.join(ROOT, "main.py")) as f:
    _msrc = f.read()
check("HELTEC-SKIP: main.py's boot-bypass pin is GPIO7, not the download-mode "
      "button", "SKIP_PIN = 7" in _msrc)

# the setup
hl.reset()
hl.uart = _UcUart()
_uart_rx[:] = b""
_ha_setup = hl.uart
hl.gps_setup()
_cmds = list(_ha_setup.cmds)
check("UC-SETUP: PDTINFO (no checksum: with one it gets no answer), then CFGNAV "
      "first (it resets the GSV rate), each with its checksum; "
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
          "CFGMSG,6,0,0", "CFGMSG,6,1,0"], repr(_cmds))
check("UC-SETUP: all answered: taken for now, the receiver's model from PDTINFO",
      hl.st["setup_ok"] is True and hl.st["checked"] is False
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
      "setup is due again and asks the model anew",
      hl.st["gps_boots"] == _b0 + 1 and hl.st["err"] == _e0
      and hl.st["extra"] >= 1 and hl.st["gps_ver"] is None,
      repr((hl.st["gps_boots"], hl.st["err"], hl.st["extra"])))

# 5 Hz with a fix: the setup's verdict
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
      "used from GSA, with its system's id; GSV is no sentence the "
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
check("UC-SETTLE-DAY: the rule does not come back after TICKS_CAP of uptime; no fix "
      "yet: nothing to wait for", _u4 is True and _nofirst and _u5 is True,
      repr((_u4, hl.st["fix_first"], _f0, _u5)))
# UC-SETTLE-STAY: the stay checks (sv_fix, hb_fix) write st directly, a receiver
# already settled (no fix_first: the rule has nothing to wait for). From the first
# fix, through the parser: its first minute standing still writes nothing and
# feeds no stay window, then the stay is proven as ever; a car's trip keeps its
# start
_se_td = tempfile.mkdtemp(prefix="settle-")
_se_saved = hl.LOG_DIR
try:
    hl.LOG_DIR = _se_td
    hl.reset()
    hl.log_reset()
    hl.wifi_reset()
    hl.up_reset()
    hl.wifi.update(state="offline", home=False)
    hl.uart = _UcUart()
    _clock[0] = (_clock[0] // 1000 + 1) * 1000
    _se_early = []
    for _i in range(hl.UC_SETTLE // 1000 - 2):
        _uc_fixes(hl, 1)
        hl.log_tick()
        _se_early.append(len(hl.log["batch"]) + bool(hl.log["still"]["win_at"]))
    for _i in range(150):
        _uc_fixes(hl, 1)
        hl.log_tick()
    _se_stay = hl.log["still"]["active"]
    hl.reset()
    hl.log_reset()
    _clock[0] = (_clock[0] // 1000 + 1) * 1000
    for _i in range(5):
        _uc_fixes(hl, 1, kn="27.0")
        hl.log_tick()
    _se_car = list(hl.log["batch"])
finally:
    hl.LOG_DIR = _se_saved
    hl.log_reset()
    hl.reset()
    shutil.rmtree(_se_td, ignore_errors=True)
check("UC-SETTLE-STAY: standing still from the first fix: nothing recorded for "
      "UC_SETTLE, then a stay as ever; at 50 km/h points from the start",
      not any(_se_early) and _se_stay is True and len(_se_car) == 1,
      repr((sum(_se_early), _se_stay, _se_car[:2])))
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
# a second queued before the restart, drained after the new first fix: it is
# older than fix_first, which must not count as UC_SETTLE gone by
_clock[0] = (_clock[0] // 1000 + 1) * 1000
_uc_fixes(hl, 2)
_q0 = hl.st["fix_first"]
hl.st.update(speed=0.0)
_q1 = hl._usable(time.ticks_add(_q0, -10000))
_q2 = hl.st["settled"]
_q3 = hl._usable(_clock[0])
check("UC-SETTLE-QUEUED: a second from before fix_first does not settle the "
      "receiver; standing still still waits",
      _q0 is not None and _q1 is False and _q2 is False and _q3 is False,
      repr((_q0, _q1, _q2, _q3)))

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
_e0 = hl.st["err"]
_uc_feed(hl, _nmea("GNGSA,A,2,30,,,,,,,,,,,,1.2,0.8,0.9"))
check("UC-GSA-OLD: a GSA without a system id (NMEA before 4.10, VDOP last) is a "
      "good sentence: its fix type taken, the list of used satellites left alone",
      hl.st["err"] == _e0 and hl.st["fixtype"] == 2 and hl._uc_sv["used"] == [30],
      repr((hl.st["err"] - _e0, hl.st["fixtype"], hl._uc_sv["used"])))

# the display: the fonts, the values, what goes to the panel
import gps_tft as _ft
import gps_tft_font as _ff
_fo = []
for _f in (_ff.CLOCK, _ff.SATS, _ff.UP, _ff.WORD, _ff.SMALL):
    _chars, _wd, _of, _dat, _h, _base = _f
    _end = 0
    _ok1 = len(_wd) == len(_chars) and len(_of) == 2 * len(_chars) and 0 < _base <= _h
    for _i in range(len(_chars)):
        _o = _of[_i * 2] | _of[_i * 2 + 1] << 8
        _ok1 = _ok1 and _o == _end
        _end = _o + ((_wd[_i] + 1) // 2) * _h
    _fo.append(_ok1 and _end == len(_dat) and len(set(_dat)) > 2)
check("TFT-FONT: every glyph's offset and size add up, 16 levels (the design's five sizes: "
      "TFT-FONT-BASE)", all(_fo), repr(_fo))

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
check("TFT-STATE: the fix; the clock; WiFi off or its "
      "level by the signal, down at once, up only past a margin; what waits, short when long, done or nothing set up; the "
      "GPS's quality from HDOP and the satellites in the fix",
      _s1[2:5] == (True, 22, "11:42") and _s1[5] is None and _s1[7:9] == ("1284", True)
      and _s2[2] is False
      and _s3[5] == 3 and _s3[7:9] == ("kész", False)
      and _lv == [3, 3, 2, 2, 1, 1, 0, 0] and _hy == [2, 2, 2, 3, 0, 3, 2, 1]
      and _s4[7] == " " and _s5[7] == "12k"
      and _gq == [3, 3, 2, 2, 2, 1, 1, 1] and _gq0 == 0 and _s1[6] == 3 and _s2[6] == 0,
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
hl.log["clock"] = ((hl._days(2026, 11, 4) - hl.EPOCH_DAYS) * 86400 + 10 * 3600 + 42 * 60 + 10,
                   _clock[0] - 60000)
check("TFT-CLOCK-RUNS: with the receiver silent the last fix's clock runs on by the "
      "ticks (as the route's day does): a minute on, 11:43",
      _ls("2026-11-04", "10:42:10") == "11:43", _ls("2026-11-04", "10:42:10"))
hl.log["clock"] = None
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
_sh1 = _store(100, (("20261103.trb", 1024 * 1024), ("20261103.trk", 1024 * 1024)))
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
      and _sf is None and _st9[9] is None
      and all(c in _ff.SMALL[0] for _p, _l in _stores for c in "%d%%" % _p + _l),
      repr((_stores, _sh, _sh0, _n1, _n2, _sf, _st9[9])))
check("TFT-STORE-PAIR: today's day written as .trk and .trb: neither counts as room",
      _sh1 == _sh0, repr((_sh1, _sh0)))

# the drawing, on a model of the panel: real pixels, at real coordinates
_order = []
_pin0 = _ft.Pin


class _BlPin(Pin):
    def __init__(self, n, *a, **k):
        super().__init__(n, *a, **k)
        if n == 21:
            _order.append((k.get("value"), len(_TftSpi.out)))


class _BlPwm(_Pwm):
    """The backlight's duty set after the panel's commands: the order, as for the Pin."""
    def duty_u16(self, d=None):
        if d is not None:
            _order.append((1 if d else 0, len(_TftSpi.out)))
        return super().duty_u16(d)


_pwm0 = _ft.PWM
_ft.Pin, _ft.PWM = _BlPin, _BlPwm
_ft._D.pwm = None
_TftSpi.out[:] = []
_TftSpi.panel.clear()
_TftSpi.fail = False
_ft.start(100, 21)
_cmds0 = list(_TftSpi.out)
_ft.Pin, _ft.PWM = _pin0, _pwm0
_ft._D.pwm = None
_PAPER_C = _ft._COL[_ft.PAPER]
_INK_C, _RED_C, _AMB_C = _ft._COL[_ft.INK], _ft._COL[_ft.RED], _ft._COL[_ft.AMBER]
_bg_all = len(_TftSpi.panel) == 160 * 80 and set(_TftSpi.panel.values()) == {_PAPER_C}
check("TFT-INIT: 16 bits a pixel and landscape, the whole panel cleared to paper, the "
      "backlight only after that (no flash of random RAM)",
      b"\x36" in _cmds0 and b"\x68" in _cmds0 and b"\x05" in _cmds0 and _bg_all
      and _order[0][0] == 0 and _order[-1][0] == 1 and _order[-1][1] == len(_cmds0),
      repr((_order, len(_cmds0), _bg_all)))

_px = lambda x, y: _TftSpi.panel.get((x + _ft.XO, y + _ft.YO))
_cnt = lambda x0, y0, x1, y1, c: sum(1 for y in range(y0, y1) for x in range(x0, x1) if _px(x, y) == c)
_rect = lambda x0, y0, x1, y1: set(_px(x, y) for y in range(y0, y1) for x in range(x0, x1))
_rgb = lambda h: _ft._c(tuple(int(h[i:i + 2], 16) for i in (1, 3, 5)))
_pal16 = lambda fg, bg: set(_ft._gs4_palette(fg, bg).pixel(v, 0) for v in range(16))
_ink15 = _ft._gs4_palette(_ft.INK).pixel(15, 0)
_red15 = _ft._gs4_palette(_ft.RED).pixel(15, 0)
_amb15 = _ft._gs4_palette(_ft.AMBER).pixel(15, 0)
_STD = (None, "8,6 km", True, 22, "11:42", None, 3, "kész", False, (5, "7 nap"))


def _show(*a, **k):
    """A frame of _STD with these fields changed, by name."""
    v = dict(zip(("route", "km", "fix", "used", "clock", "level", "quality", "up", "up_on", "store"), _STD))
    v.update(k)
    _ft.set_state(*[v[n] for n in ("route", "km", "fix", "used", "clock", "level", "quality", "up", "up_on", "store")])
    _ft.frame()


_show()
check("TFT-COLOURS: exactly the design's: paper, ink, red, amber, unlit dots, off bars, the "
      "four tiles", (_PAPER_C, _INK_C, _RED_C, _AMB_C, _ft.DIM, _ft.OFF)
      == tuple(_rgb(h) for h in ("#f4f0e6", "#111111", "#d7261e", "#c98a00", "#e2dccb", "#cfc9ba"))
      and [_ft._COL[t] for _w, t in _ft.QUALITY]
      == [_rgb(h) for h in ("#d7261e", "#d9601a", "#c98a00", "#1f8a4c")]
      and [w for w, _t in _ft.QUALITY] == ["nincs", "gyenge", "közepes", "erős"])

# the right column
_clk = _rect(96, 0, 160, 15) - {_PAPER_C}
_clk_ok = _clk <= _pal16(_ft.INK, _ft.PAPER) and _ink15 in _clk
_clk_end = max(x for x in range(96, 160) for y in range(0, 15) if _px(x, y) != _PAPER_C)
_clk_base = max(y for x in range(96, 160) for y in range(0, 15) if _px(x, y) != _PAPER_C)
_up_end = max(x for x in range(96, 160) for y in range(48, 63) if _px(x, y) != _PAPER_C)
_up_base = max(y for x in range(96, 160) for y in range(48, 63) if _px(x, y) != _PAPER_C)
_rule = [_px(x, 47) for x in range(95, 160)]
check("TFT-COLUMN: the clock's baseline 14 and the upload's 62, both ending at x=158 at "
      "most, ink anti-aliased on paper; the rule at y=47 from x=96 to the edge",
      _clk_ok and _clk_base == 13 and 155 <= _clk_end < 158 and _up_base == 61 and 155 <= _up_end < 158
      and _rule[0] == _PAPER_C and set(_rule[1:]) == {_INK_C}
      and set(_px(x, 46) for x in range(96, 160)) == {_PAPER_C},
      repr((_clk_base, _clk_end, _up_base, _up_end)))

# the satellites: an icon left of the count, on its baseline, in its colour
_SAT = ("...#####...", "....###....", "###..#..###", "###.###.###", "###.###.###", "###########",
        "###.###.###", "###.###.###", "###..#..###", "###..#..###", "....###....")


def _sat_at(n):
    """The icon's left edge for count n, from the font's own width."""
    return 158 - _ft.width(_ft._SATS, str(n)) - _ft.SAT_GAP - 11


def _sat_ok(n, col):
    x0 = _sat_at(n)
    return all(_px(x0 + i, 17 + j) == (col if ch == "#" else _PAPER_C)
               for j, row in enumerate(_SAT) for i, ch in enumerate(row))


_sat1 = _sat_ok(22, _INK_C)
_num1 = _rect(_sat_at(22) + 11, 17, 160, 28) - {_PAPER_C}
_sat_end = max(x for x in range(_sat_at(22) + 11, 160) for y in range(17, 28) if _px(x, y) != _PAPER_C)
_gap1 = _rect(_sat_at(22) + 11, 17, _sat_at(22) + 13, 28)
_show(fix=False, used=0, clock="--:--")
_sat2 = _sat_ok(0, _RED_C)
_num2 = _rect(_sat_at(0) + 11, 17, 160, 28) - {_PAPER_C}
_show(used=40)
_sat3 = _sat_ok(40, _INK_C)
_num40 = _ft.width(_ft._SATS, "40")
_clear = _rect(92, 14, 160, 17) | _rect(92, 28, 160, 31)
check("TFT-SAT: the icon, pixel by pixel, 11x11 on the count's baseline, left of it with a "
      "gap and no overlap, ink, red without a fix with the count; the widest count keeps "
      "clear of the clock, the bars and the km; no \"sat\" text in the font",
      _sat1 and _sat2 and _sat3 and _num1 <= _pal16(_ft.INK, _ft.PAPER)
      and _num2 <= _pal16(_ft.RED, _ft.PAPER) and _red15 in _num2 and _ink15 in _num1
      and _gap1 == {_PAPER_C} and 2 <= _ft.SAT_GAP <= 3 and _sat_end == 157
      and _clear == {_PAPER_C} and _sat_at(40) >= 96 and _num40 < 20
      and "a" not in _ff.SATS[0] and "t" not in _ff.SATS[0],
      repr((_sat1, _sat2, _sat3, _sat_end, _gap1, _clear)))

# WiFi: four rising bars
_bars = []
for _lv in (None, 0, 1, 2, 3):
    _show(level=_lv)
    _bars.append([(sum(1 for y in range(28, 47) if _px(141 + 5 * i, y) == _INK_C),
                   sum(1 for y in range(28, 47) if _px(141 + 5 * i, y) == _ft.OFF)) for i in range(4)])
_bar_w = [_px(x, 41) for x in range(138, 160)]
_bar_foot = set(_px(x, 42) for x in range(140, 160))
check("TFT-BARS: x=140, foot y=42, heights 2 5 8 11, none lit without WiFi, level 0-3 lights "
      "that many plus one; 4 wide on a pitch of 5, ending at x=158",
      _bars == [[(0, 2), (0, 5), (0, 8), (0, 11)], [(2, 0), (0, 5), (0, 8), (0, 11)],
                [(2, 0), (5, 0), (0, 8), (0, 11)], [(2, 0), (5, 0), (8, 0), (0, 11)],
                [(2, 0), (5, 0), (8, 0), (11, 0)]]
      and _bar_w == [_PAPER_C] * 2 + ([_INK_C] * 4 + [_PAPER_C]) * 4
      and _bar_foot == {_PAPER_C}, repr((_bars, _bar_w)))

# the tile and the flash's gauge
_tiles = []
for _q, (_w, _t) in enumerate(_ft.QUALITY):
    _show(quality=_q)
    _c0 = _ft._COL[_t]
    _inner = _rect(0, 64, 62, 80)
    _ws = [x for x in range(62) if any(_px(x, y) != _c0 for y in range(64, 80))]
    _tiles.append((_px(0, 64) == _c0 and _px(61, 79) == _c0 and _px(0, 63) == _PAPER_C
                   and _px(62, 70) == _PAPER_C,
                   _inner <= _pal16(_ft.PAPER, _t),
                   _ft._gs4_palette(_ft.PAPER, _t).pixel(15, 0) in _inner,
                   abs(_ws[0] + _ws[-1] - 61) <= 3))
_wmax = _ft.width(_ft._WORD, "közepes")
check("TFT-TILE: 0,64 62x16 in the quality's colour, its word centred in paper, every pixel "
      "blended against that tile; the longest word fits",
      all(all(t) for t in _tiles) and _wmax <= 60, repr((_tiles, _wmax)))

_gauge = []
for _p, _l, _len, _col, _tone in ((3, "7 nap", 2, _INK_C, _ink15), (50, "3 nap", 46, _INK_C, _ink15),
                                  (70, "44 óra", 64, _AMB_C, _amb15), (89, "9 óra", 81, _AMB_C, _amb15),
                                  (90, "8 óra", 82, _RED_C, _red15), (100, "tele", 92, _RED_C, _red15)):
    _show(store=(_p, _l))
    _g = [[_px(x, y) for x in range(66, 158)] for y in (75, 76, 77)]
    _txt = _rect(64, 64, 160, 74) - {_PAPER_C}
    _gauge.append(all(r[:_len] == [_col] * _len and r[_len:] == [_ft.DIM] * (92 - _len) for r in _g)
                  and _px(65, 76) == _PAPER_C and _px(158, 76) == _PAPER_C
                  and set(_px(x, 74) for x in range(66, 158)) == {_PAPER_C}
                  and set(_px(x, 78) for x in range(66, 158)) == {_PAPER_C}
                  and _tone in _txt)
_store_l = max(x for x in range(160) if _px(x, 70) not in (_PAPER_C, None))
_show(store=(0, "7 nap"))
_g0 = _px(66, 76) == _INK_C and _px(67, 76) == _ft.DIM
_small_base = max(y for x in range(66, 110) for y in range(64, 74) if _px(x, y) != _PAPER_C)
_show(store=None)
_none = _rect(64, 64, 160, 80) == {_PAPER_C}
check("TFT-GAUGE: 66,75 92x3 on its unlit colour, the length by the per cent (at least one "
      "pixel), the bar and the text ink, amber from 70, red from 90; unreadable: nothing",
      all(_gauge) and _g0 and _none and _small_base == 71, repr((_gauge, _g0, _none, _small_base)))

# the strings, each in its font, the longest in their slots
_str_small = ["%d%%" % p for p in (0, 5, 96, 100)] + [hl._store_left(b) for b in (
    0, 1, hl.TFT_POINT_B * 3600, hl.TFT_POINT_B * 3600 * 47, hl.TFT_POINT_B * 3600 * 48 * 9)]
_str_km = ["0,0 km", "8,6 km", "99,9 km", "100 km", "999 km", "1234567890 km"]
check("TFT-CHARS: every string the screen shows has its characters in its font",
      all(c in _ff.CLOCK[0] for c in "0123456789:-")
      and all(c in _ff.SATS[0] for c in "0123456789")
      and all(c in _ff.UP[0] for s in ("kész", "0123456789", "12k", "999k") for c in s)
      and all(c in _ff.WORD[0] for w, _t in _ft.QUALITY for c in w)
      and all(c in _ff.SMALL[0] for s in _str_small + _str_km + [_ft.ROUTE_NOTE] for c in s)
      and _ft.ROUTE_NOTE == "nincs út",
      repr(_str_small))
# a character no font has ('?' is in none): a blank half the font's height wide,
# the glyphs around it where they would be; not glyph 0 (a digit in its place)


def _band_text(s, x):
    fb = _FakeFramebuf.FrameBuffer(bytearray(160 * _ft.BAND_H * 2), 160, _ft.BAND_H, _FakeFramebuf.RGB565)
    fb.fill(_PAPER_C)
    return fb, _ft._text(fb, _ft._SATS, s, x, 12, _ft.INK)


_unk_fb, _unk_x = _band_text("1?2", 10)
_unk_gap = (_ft._SATS[0][4] + 1) // 2
_ref_fb, _ref_x = _band_text("1", 10)
_ft._text(_ref_fb, _ft._SATS, "2", 10 + _ft.width(_ft._SATS, "1") + _unk_gap, 12, _ft.INK)
_glyph0_fb, _g0 = _band_text("102", 10)
check("TFT-UNKNOWN: a character the font lacks is a blank (half the height), the "
      "string's width and the next glyph's place count it; glyph 0 is not drawn for it",
      all("?" not in f[0] for f in (_ff.CLOCK, _ff.SATS, _ff.UP, _ff.WORD, _ff.SMALL))
      and _ft._index(_ft._SATS[0], "?") == -1 and _unk_gap > 0
      and _ft.width(_ft._SATS, "1?2") == _ft.width(_ft._SATS, "1") + _unk_gap + _ft.width(_ft._SATS, "2")
      and _unk_x == 10 + _ft.width(_ft._SATS, "1?2")
      and bytes(_unk_fb.buf) == bytes(_ref_fb.buf) and bytes(_unk_fb.buf) != bytes(_glyph0_fb.buf),
      repr((_unk_gap, _unk_x, _ft.width(_ft._SATS, "1?2"))))
_sw = _ft.width
_store_room = _sw(_ft._SMALL, "100%") + _sw(_ft._SMALL, "<1 óra")
_up_room = _sw(_ft._UP, "12345")
check("TFT-FIT: the longest strings fit their slots: 100% with <1 óra over the gauge, a "
      "12345 count and --:-- in the right column",
      _store_room < 92 - 2 and _up_room < 158 - 96 and _sw(_ft._CLOCK, "--:--") < 158 - 96
      and _sw(_ft._CLOCK, "88:88") < 158 - 96, repr((_store_room, _up_room)))
_show(store=(100, "<1 óra"), up="12345", clock="--:--")
_long = (_rect(92, 48, 96, 63) == {_PAPER_C} and _px(158, 72) == _PAPER_C and _rect(158, 48, 160, 63) == {_PAPER_C})

# what goes up when: a zone of 4 bands (rows 0-62) or 2 (rows 63-79), or nothing
_show()


_zcur = {}


def _sends(**k):
    _zcur.update(k)
    _TftSpi.frames = 0
    _TftSpi.out[:] = []
    _show(**_zcur)
    return _TftSpi.frames


_z = dict(same=_sends(), same2=_sends(), up_on=_sends(up_on=True), km=_sends(km="8,7 km"),
          route=_sends(km="8,7 km", route=(bytearray(4), 2, 0, 1)),
          route_same=_sends(km="8,7 km", route=(bytearray(4), 2, 0, 1)),
          used=_sends(used=23), level=_sends(level=2), level_same=_sends(level=2),
          up=_sends(up="123"), clock=_sends(clock="11:43"),
          fix=_sends(fix=False), quality=_sends(quality=1), store=_sends(store=(6, "7 nap")))
_TftSpi.frames = 0
_show(route=None, km="", used=7, quality=2, store=(7, "6 nap"))
_both = _TftSpi.frames
_show()
_TftSpi.out[:] = []
for _i in range(20):
    _ft.frame()
_still = sum(1 for b in _TftSpi.out if len(b) > 8) == 0
check("TFT-ZONES: an unchanged state sends nothing, for 20 frames too; the route, km, "
      "satellites, WiFi level, clock, fix and upload redraw the top zone only (4 bands), "
      "quality and flash the low zone only (1 band); up_on alone nothing",
      _z == dict(same=0, same2=0, up_on=0, km=4, route=4, route_same=0, used=4, level=4,
                 level_same=0, up=4, clock=4, fix=4, quality=1, store=1)
      and _both == 5 and _still and _long, repr((_z, _both, _still, _long)))
# the low zone is rows 64-79: one whole band to the panel's rows YO+64..YO+79
# (it was rows 63-79: a 16-row band and a second one of a single row); the top
# zone's last band ends at row 63, paper in both zones
_sends()                                 # (the panel at _zcur again: _show() drew _STD)
_sends(store=(9, "5 nap"))
_low_data = [len(b) for b in _TftSpi.out if len(b) > 8]
_low_win = [bytes(b) for b in _TftSpi.out if len(b) == 4]
_sends(clock="11:44")
_top_win = [bytes(b) for b in _TftSpi.out if len(b) == 4]
_top_data = [len(b) for b in _TftSpi.out if len(b) > 8]
_rows = lambda a, b: bytes((a >> 8, a & 255, b >> 8, b & 255))   # a 0x2B window (">HH")
check("TFT-LOWBAND: the low zone goes up as one 16-row band (rows 64-79), the top "
      "zone's last band ends at row 63; no 1-row band",
      _low_data == [160 * 16 * 2] and _low_win.count(_rows(_ft.YO + 64, _ft.YO + 79)) == 1
      and _rows(_ft.YO + 48, _ft.YO + 63) in _top_win and _top_data == [160 * 16 * 2] * 4
      and _ft.LOW_Y == 64 and not hasattr(_ft, "TOP_H") and not hasattr(_ft, "TILE_Y")
      and _rect(0, 63, 160, 64) == {_PAPER_C},
      repr((_low_data, _low_win, _top_data)))

# today's route: the logger's side
M_E6 = hl._CM_E6 / 100
_rt_dir = os.path.join(diag_dir, "route")
os.makedirs(_rt_dir, exist_ok=True)
hl.ROUTE_FILE = os.path.join(_rt_dir, "route.bin")
_DAY = (hl._days(2026, 6, 15) - hl.EPOCH_DAYS) * 86400 + 36000   # 12:00 Hungarian
_Rt = hl._Rt


def _rfresh():
    _rt_rm = [hl.ROUTE_FILE, hl.ROUTE_FILE + ".tmp"]
    for _n in _rt_rm:
        try:
            os.remove(_n)
        except OSError:
            pass
    hl._rt_start()
    hl._rt_reset(-1)
    _Rt.view, _Rt.km, _Rt.tail, _Rt.drawn, _Rt.saved = None, "", None, None, None
    _Rt.hour, _Rt.moved, _Rt.unsaved, _Rt.waiting, _Rt.save_ok = None, False, False, False, True
    hl._tft = _ft
    hl._tft_t[:] = [None, None, 0, 0, 0]


def _rline(lat, lon, s, anchor=False):
    """A track line as _sample_line and _still_heartbeat write them, format 1 (an
    anchor's time noted in log["hb"], as _still_heartbeat does)."""
    tc = hl._stamp_s(s)[1]
    if anchor:
        hl.log.setdefault("hb", []).append(tc)
        return "%.6f,%.6f,,,%s\n" % (lat, lon, tc)
    return "%.6f,%.6f,120.5,1.0,%s\n" % (lat, lon, tc)


def _rfeed(*lines):
    hl._rt_feed(list(lines), len(lines))


def _rat(east_m, north_m, s, anchor=False, lat0=10.0, lon0=20.0):
    """A line that many metres from the made-up origin."""
    lat = lat0 + north_m / M_E6 / 1e6
    return _rline(lat, lon0 + east_m / M_E6 / 1e6 / math.cos(math.radians(lat0)), s, anchor)


def _rview(s=None):
    """A look at the route, the clock at s."""
    _clock[0] += hl.ROUTE_DRAW
    if s is not None:
        hl.log["clock"] = (s, _clock[0])
    hl._rt_tick(_clock[0])
    return _Rt.view


hl.log_reset()
hl.st.update(valid=False, pos_at=None)

# A real day's route, its shape only: moved to a made-up origin and date, thinned.
# (east dm, north dm, s, an anchor point): two trips, the second starting at an anchor
# with the 44 m jump after it, as it did on the board.
_ROUTE_DAY = [
  [
    (0,0,0,0),(60,-18,2,0),(150,13,4,0),(568,207,14,0),(679,80,17,0),(1053,-592,35,0),
    (1199,-592,39,0),(1563,-458,44,0),(9705,3451,109,0),(13688,5275,138,0),(14092,5411,141,0),
    (15002,5497,148,0),(15194,5542,150,0),(15457,5653,155,0),(16011,6011,337,0),
    (16020,6056,344,0),(15957,6134,346,0),(15324,6516,355,0),(14772,7085,396,0),
    (14509,7495,403,0),(14337,7904,408,0),(14273,8171,411,0),(14238,8518,415,0),
    (14211,9816,429,0),(14115,11484,489,0),(14002,11916,493,0),(13892,12138,495,0),
    (13741,12349,497,0),(13334,12672,501,0),(8138,15990,545,0),(6494,17153,559,0),
    (6163,17426,562,0),(5037,18521,573,0),(4479,19157,579,0),(3332,20567,594,0),
    (3341,20689,597,0),(3426,20756,599,0),(4000,20811,605,0),(4447,20905,609,0),
    (4670,20989,611,0),(5387,21367,618,0),(7095,21690,644,0),(8831,21868,659,0),
    (8886,21892,660,0),(8932,21994,662,0),(8793,23044,682,0),(8762,23120,700,0),
    (8481,23164,754,0),(8274,23057,775,0),(7807,22750,820,0),(7788,22617,837,0),
    (7743,22657,855,0),(7785,22688,861,0),(7795,22535,884,0),
  ],
  [
    (7856,22230,5588,1),(8029,21825,5589,0),(6840,21696,5599,0),(6087,21558,5606,0),
    (5952,21567,5608,0),(5812,21696,5612,0),(5748,21705,5613,0),(5594,21645,5615,0),
    (4736,21100,5624,0),(4236,20922,5629,0),(3717,20878,5635,0),(2790,20878,5674,0),
    (2238,20811,5679,0),(1939,20687,5682,0),(597,19904,5696,0),(-601,19272,5709,0),
    (-898,19148,5714,0),(-1108,19139,5718,0),(-1177,19054,5720,0),(-1127,18961,5722,0),
    (4440,14838,5780,0),(4407,14753,5782,0),(1823,12183,5815,0),(1662,11907,5818,0),
    (1477,11262,5825,0),(1401,11093,5828,0),(1431,10993,5830,0),(1693,10904,5834,0),
    (3172,10572,5850,0),(3422,10497,5896,0),(3481,10452,5898,0),(3493,10370,5900,0),
    (3422,10281,5902,0),(2421,9807,5913,0),(2368,9770,5914,0),(2333,9681,5916,0),
    (3660,6676,5961,0),(3612,6605,5963,0),(1191,4844,6024,0),(1182,4766,6026,0),
    (1211,4710,6027,0),(1743,3816,6038,0),(1746,3683,6040,0),(1519,3438,6044,0),
    (-360,1993,6072,0),(-342,1855,6075,0),(467,320,6097,0),(402,229,6100,0),(79,89,6107,0),
    (346,267,6209,0),
  ],
]


def _rday():
    """The fixture's lines, trip by trip, and the km of its real lines, by hand."""
    out, m = [], 0.0
    for trip in _ROUTE_DAY:
        lines = [_rat(e / 10, n / 10, _DAY + t, a == 1) for e, n, t, a in trip]
        real = [r for r in trip if not r[3]]
        for a, b in zip(real, real[1:]):
            m += math.hypot(b[0] - a[0], b[1] - a[1]) / 10
        out.append(lines)
    return out, m


_rfresh()
_trips, _hand = _rday()
for _tr in _trips:
    _rfeed(*_tr)
_v = _rview(_DAY + 7000)
_cap_ok = _Rt.n <= hl.ROUTE_CAP and len(_Rt.pts) == 2 * hl.ROUTE_CAP and len(_Rt.xy) == 2 * hl.ROUTE_CAP + 2
_rx = [_v[0][2 * i] for i in range(_v[1])]
_ry = [_v[0][2 * i + 1] & 127 for i in range(_v[1])]
_flags = [i for i in range(_v[1]) if _v[0][2 * i + 1] & 128]
_e = [r for t in _ROUTE_DAY for r in t if not r[3]]
_real_w = (max(r[0] for r in _e) - min(r[0] for r in _e)) / 10
_real_h = (max(r[1] for r in _e) - min(r[1] for r in _e)) / 10
_sc = (max(_ry) - min(_ry)) / _real_h
check("ROUTE-REPLAY: a real day's points give 8,6 km without its anchor's 44 m, two trips, "
      "inside the buffer and the field: fitted, centred, north up, the true aspect",
      _Rt.km == "8,6 km" and abs(_Rt.cm / 100 - _hand) < 3 and _cap_ok and len(_flags) == 2
      and _flags[0] == 0 and _v[2] == _flags[1]
      and 0 <= min(_rx) and max(_rx) <= hl.ROUTE_W and 0 <= min(_ry) and max(_ry) <= hl.ROUTE_H
      and max(_ry) - min(_ry) == hl.ROUTE_H
      and abs((max(_rx) - min(_rx)) / _real_w / _sc - 1) < 0.05
      and abs(min(_rx) + max(_rx) - hl.ROUTE_W) <= 2
      and _ry[0] > _ry[_flags[1] - 1],           # the way out goes north: up
      repr((_Rt.km, _Rt.cm, _hand, _Rt.n, _flags, _v[2], min(_rx), max(_rx), min(_ry), max(_ry))))

# anchors add nothing, and neither does a false jump next to one
_rfresh()
_rfeed(_rat(0, 0, _DAY), _rat(100, 0, _DAY + 10))
_c1, _n1 = _Rt.cm, _Rt.n
_rfeed(_rat(5000, 0, _DAY + 20, True))
_c2, _n2 = _Rt.cm, _Rt.n
_rfeed(_rat(200, 0, _DAY + 30))
_c3 = _Rt.cm
check("ROUTE-ANCHOR: an anchor point 5 km off adds no km and no point, and the next real "
      "line counts from the last real one",
      abs(_c1 - 10000) < 3 and _c2 == _c1 and _n2 == _n1 and abs(_c3 - 20000) < 5, repr((_c1, _c2, _c3)))
_rfresh()
_rfeed(_rat(0, 0, _DAY))
hl._rt_trip(_DAY + 100)
_rfeed(_rat(0, 0, _DAY + 90, True), _rat(44, 0, _DAY + 100), _rat(55, 0, _DAY + 101))
check("ROUTE-JUMP: a stay's end: the anchor and the way out's first line 44 m on (158 km/h, "
      "measured) count nothing, only the way out itself; a new trip",
      abs(_Rt.cm - 1100) < 15 and _Rt.pts[2] & 1 == 1, repr((_Rt.cm, _Rt.n)))

# a fix of a second without GGA has no height and no HDOP either: it is track,
# only the logger's own anchor points (told by their time) add nothing
_rfresh()
hl.log_reset()
_rfeed(_rat(0, 0, _DAY), _rat(0, 55, _DAY + 1).replace(",120.5,1.0,", ",,,"),
       _rat(0, 110, _DAY + 2).replace(",120.5,1.0,", ",,,"), _rat(0, 165, _DAY + 3))
_rmc = (_Rt.n, _Rt.cm)
_rfeed(_rat(5000, 0, _DAY + 20, True))
check("ROUTE-RMC-ONLY: a line without height and HDOP (a second with RMC only) is a "
      "point of the route like any other, 3 x 55 m; an anchor point is told by its time",
      _rmc[0] == 4 and abs(_rmc[1] - 16500) < 20 and (_Rt.n, _Rt.cm) == _rmc
      and hl.log.get("hb") == [], repr((_rmc, _Rt.n, _Rt.cm, hl.log.get("hb"))))

# a heartbeat whose stamp would not follow the last line waits a second, not a pass
hl.log_reset()
_sl = hl.log["still"]
_sl.update(active=True, anchor=(10.5, 20.5))
hl.log["clock"] = (_DAY, _clock[0])
hl.log["last_t"] = hl._stamp_s(_DAY + 30)[0]
hl._still_heartbeat(_clock[0])
_hb_wait = (len(hl.log["batch"]), _sl["hb_at"] is not None
            and hl._recent(_clock[0] + 999, _sl["hb_at"], hl.STILL_HEARTBEAT)
            and not hl._recent(_clock[0] + 1000, _sl["hb_at"], hl.STILL_HEARTBEAT))
hl.log["last_t"] = None
hl._still_heartbeat(_clock[0] + 1000)
check("ROUTE-HB-WAIT: an anchor point whose time would not follow the last line is not "
      "written, and the next try is a second on, not every pass; then it is written",
      _hb_wait == (0, True) and len(hl.log["batch"]) == 1 and _sl["hb_at"] == _clock[0] + 1000,
      repr((_hb_wait, len(hl.log["batch"]), _sl["hb_at"], _clock[0])))
hl.log_reset()

# through the logger itself: only what flush() really wrote counts
_rfresh()
hl.log_reset()
hl.st.update(date="2026-06-15", time="10:00:00", valid=True, lat=10.0, lon=20.0, pos_at=_clock[0],
             rx=_clock[0], alt=120.5, hdop=1.0, hdop_at=_clock[0], fixtype=3)


def _rrec(east, s):
    p = (10.0, 20.0 + east / M_E6 / 1e6 / math.cos(math.radians(10.0)))
    tc = hl._stamp_s(s)[1]
    hl._record((hl._stamp_s(s)[0], hl._line("%.6f,%.6f" % p, ("120.5", "1.0"), tc)), p)


for _i in range(5):
    _rrec(_i * 20, _DAY + _i)
_held = _Rt.cm                           # in RAM only: nothing yet
hl.flush(None)
_wrote = _Rt.cm
_rrec(120, _DAY + 5)
_fgood = hl.log["file"]
hl.log["file"] = os.path.join(_rt_dir, "no", "such", "dir.trb")
hl.log["created"] = False
hl.flush(None)
_failed = (_Rt.cm, len(hl.log["batch"]))
hl.log["file"] = _fgood
hl.log["created"] = True
hl.log["wfail"] = False
hl.flush(None)
_after = _Rt.cm
# an anchor point written by the logger
_sl = hl.log["still"]
_sl.update(anchor=(10.5, 20.5), active=True)
hl.log["clock"] = (_DAY + 100, _clock[0])
hl._still_heartbeat(_clock[0])
_hb = len(hl.log["batch"])
hl.flush(None)
_hb_cm = (_Rt.cm, _Rt.n, len(hl.log["batch"]))
hl.log["still"] = hl._still_state()
check("ROUTE-FLUSH: lines count when flush() wrote them, not when recorded nor when the "
      "write failed; the logger's own anchor point goes through and adds nothing",
      _held == 0 and abs(_wrote - 8000) < 30 and _failed == (_wrote, 1) and abs(_after - 12000) < 40
      and _hb == 1 and _hb_cm[0] == _after and _hb_cm[2] == 0,
      repr((_held, _wrote, _failed, _after, _hb, _hb_cm)))

# the drift a stay's proof drops never counted
_rfresh()
hl.log_reset()
hl.log["clock"] = (_DAY, _clock[0])
for _i in range(10):
    _rrec(_i * 50, _DAY + _i)
for _i in range(40):                     # standing at 450 m, 20 m of drift a line
    _rrec(450 + (20 if _i % 2 else 0), _DAY + 10 + _i)
_anchor = (10.0, 20.0 + 450 / M_E6 / 1e6 / math.cos(math.radians(10.0)))
hl._still_enter(_clock[0], _anchor, hl._stamp_s(_DAY + 10)[1], 40)
hl.flush(None)
_drift_cm = _Rt.cm
hl.log["still"] = hl._still_state()
check("ROUTE-DRIFT: a stay's proof drops the drift from the track: 800 m of it adds nothing "
      "(only the way in, 450 m)", 44000 < _drift_cm < 47000, repr(_drift_cm))

# a failure here must not stop the logging
_rfresh()
hl.log_reset()
_dl0, _dw0 = [], hl._diag_write
hl._diag_write = _dl0.append
_rl0 = hl._rt_line


def _rboom(line):
    raise ValueError("route")


hl._rt_line = _rboom
_rok = []
for _i in range(hl.TFT_MAX_ERR):
    _rrec(_i * 10, _DAY + _i)
    hl.flush(None)
    _rok.append((len(hl.log["batch"]), hl.log["points"]))
hl._rt_line = _rl0
hl._diag_write = _dw0
check("ROUTE-FAIL: a route that raises leaves each line written and gone from the batch; it "
      "counts as the display's errors do: TFT_MAX_ERR in a row and the display goes dark",
      _rok == [(0, 1), (0, 2), (0, 3)] and hl._tft is None and len(_dl0) == 1
      and _dl0[0].startswith("bug tft ")
      and [p[:2] for p in _pin_levels if p[0] == 21][-1:] == [(21, 0)], repr((_rok, _dl0)))
_rfresh()
_rm0 = hl._rt_project


def _rboom2():
    raise MemoryError()


hl._rt_project = _rboom2
_rfeed(_rat(0, 0, _DAY))
_st_ok = True
try:
    hl._tft_state(_clock[0] + hl.ROUTE_DRAW)
except Exception:
    _st_ok = False
_errs = hl._tft_t[4]
hl._rt_project = _rm0
hl._tft_t[4] = 0
check("ROUTE-FAIL-TICK: a tick that raises is caught and counted, the values still come",
      _st_ok and _errs == 1, repr((_st_ok, _errs)))

# the Hungarian day
_rfresh()
_w0 = (hl._days(2026, 11, 4) - hl.EPOCH_DAYS) * 86400 + 22 * 3600 + 59 * 60 + 50   # 23:59:50 CET
_rfeed(_rat(0, 0, _w0 - 10), _rat(100, 0, _w0))
_before = _Rt.km, _Rt.cm > 0, _Rt.n
_d0 = _Rt.day
_rfeed(_rat(200, 0, _w0 + 20))           # 00:00:10
_after = _Rt.cm, _Rt.n, _Rt.day
_rfresh()
_rfeed(_rat(0, 0, _w0 - 10), _rat(100, 0, _w0))
_rview(_w0)
_stays = _Rt.cm > 0
_rview(_w0 + 20)                         # midnight: no line needed
_tick = _Rt.cm, _Rt.n, _Rt.km, _Rt.view[1]
_rfeed(_rat(100, 0, _w0))                # held over midnight: yesterday's
_old = _Rt.cm, _Rt.n
_rfresh()
_s0 = (hl._days(2026, 7, 1) - hl.EPOCH_DAYS) * 86400 + 21 * 3600 + 59 * 60 + 50    # 23:59:50 CEST
_rfeed(_rat(0, 0, _s0 - 10), _rat(100, 0, _s0))
_sum1 = _Rt.cm
_rfeed(_rat(150, 0, _s0 + 5))            # 23:59:55, no new day yet
_sum2 = _Rt.cm
_rfeed(_rat(200, 0, _s0 + 20))
_sum3 = _Rt.cm, _Rt.n
_ut = hl._rt_day(_w0) + 1 == hl._rt_day(_w0 + 20) and hl._rt_day(_s0) + 1 == hl._rt_day(_s0 + 20)
check("ROUTE-MIDNIGHT: reset at local midnight, CET and CEST (not at UTC's): a line of the "
      "new day or the clock alone, km and points gone; a held line of yesterday changes nothing",
      _after == (0, 1, _d0 + 1) and _stays and _tick == (0, 0, "0,0 km", 0) and _old == (0, 0)
      and _sum1 > 0 and _sum2 > _sum1 and _sum3 == (0, 1) and _ut and _before[1],
      repr((_before, _after, _tick, _old, _sum1, _sum2, _sum3)))

# halving: any day fits
_rfresh()
_pts0, _xy0 = _Rt.pts, _Rt.xy
_ns, _steps = [], []
_rs = _DAY
for _i in range(3000):
    _rfeed(_rat(_i * 200, (_i % 7) * 40, _rs + _i * 10))
    _ns.append(_Rt.n)
    _steps.append(_Rt.step)
_first = _Rt.pts[0] >> 1, _Rt.pts[1]
_last = _Rt.pts[2 * (_Rt.n - 1)] >> 1, _Rt.pts[2 * (_Rt.n - 1) + 1]
_want_last = _rat(2999 * 200, (2999 % 7) * 40, 0).split(",")
_want_first = _rat(0, 0, 0).split(",")
_keep = (_Rt.pts is _pts0 and _Rt.xy is _xy0 and len(_pts0) == 2 * hl.ROUTE_CAP)
_v = _rview(_rs + 30000)
_rx = [_v[0][2 * i] for i in range(_v[1])]
_mk = sum(1 for i in range(_Rt.n) if _Rt.pts[2 * i] & 1)
_rfresh()
for _i in range(hl.ROUTE_CAP):
    _Rt.pts[2 * _i] = (47000000 + _i) << 1
    _Rt.pts[2 * _i + 1] = 18000000 + _i
_Rt.n = hl.ROUTE_CAP
_Rt.pts[2 * 5] |= 1                      # a mark on a point that goes
_Rt.pts[2 * 63] |= 1                     # and on the last
_hstep = _Rt.step
hl._rt_halve()
_h = [(_Rt.pts[2 * _i] >> 1, _Rt.pts[2 * _i + 1], _Rt.pts[2 * _i] & 1) for _i in range(_Rt.n)]
_h_ok = (_Rt.n == 33 and _h[0] == (47000000, 18000000, 0) and _h[-1] == (47000063, 18000063, 1)
         and _Rt.step == 2 * _hstep and [m for _a, _b, m in _h].count(1) == 2 and _h[3] == (47000006, 18000006, 1)
         and all(_h[_i][0] == 47000000 + 2 * _i for _i in range(32)))
for _i in range(40):                     # however often: a spacing the file can hold
    _Rt.n = hl.ROUTE_CAP
    hl._rt_halve()
_h_cap = _Rt.step <= 2 * hl.ROUTE_STEP_MAX and _Rt.step > 1 << 20
_Rt.day, _Rt.last_s = 1, 1
hl._rt_save(_clock[0])
_h_cap = _h_cap and hl._rt_load()
_rfresh()
_rs2 = _DAY
_tr_marks = []
for _t in range(8):                      # short trips: their marks must outlive halving
    hl._rt_trip(_rs2)
    for _i in range(100):
        _rfeed(_rat(_t * 30000 + _i * 200, 0, _rs2))
        _rs2 += 10
_marks = [i for i in range(_Rt.n) if _Rt.pts[2 * i] & 1]
_sv = _rview(_rs2)
check("ROUTE-HALVE: never past the buffer, the same one for the whole day, the spacing "
      "doubling; the first point and the last position stay; the km still add up; the trips' "
      "marks survive, moved on to the next point kept",
      _h_ok and _h_cap and max(_ns) == hl.ROUTE_CAP and _keep and _steps[-1] > _steps[0] and _steps[-1] % hl.ROUTE_STEP == 0
      and all(b in (a, 2 * a) for a, b in zip(_steps, _steps[1:]))
      and _first == (round(float(_want_first[0]) * 1e6), round(float(_want_first[1]) * 1e6))
      and _last == (round(float(_want_last[0]) * 1e6), round(float(_want_last[1]) * 1e6))
      and _mk == 1 and min(_rx) >= 0 and max(_rx) <= hl.ROUTE_W
      and _rx[0] == min(_rx) and _rx[-1] == max(_rx) and len(_marks) == 8 and _marks[0] == 0
      and _sv[2] == _marks[-1] and abs(_Rt.cm - 8 * 99 * 20000) < 8 * 99 * 3,
      repr((_h_ok, max(_ns), _steps[-1], _first, _last, _mk, _marks, _Rt.cm)))

# trips: a stay's end, a fix gap's new file; the time between lines
_rfresh()
hl.log_reset()
_rfeed(_rat(0, 0, _DAY), _rat(100, 0, _DAY + 60))
_rfeed(_rat(200, 0, _DAY + 60 + hl.NEW_FILE_GAP))      # exactly the gap: same trip
_same = _Rt.cm, sum(_Rt.pts[2 * i] & 1 for i in range(_Rt.n))
_rfeed(_rat(5000, 0, _DAY + 61 + 2 * hl.NEW_FILE_GAP))  # past it: a new one
_new = _Rt.cm, sum(_Rt.pts[2 * i] & 1 for i in range(_Rt.n))
_sl = hl.log["still"]
_sl.update(active=True, anchor=(10.0, 20.0), aref=(10.0, 20.0), rn=0, ri=0)
_Rt.trip = None
hl.st.update(date="2026-06-15", time="11:30:00")
hl._still_leave()
_leave = _Rt.trip
_Rt.trip = None
hl.log["file"] = "x"
hl.log.update(fix=False, lost_at=_clock[0] - hl.NEW_FILE_GAP * 1000 - 1, created=True)
hl.st.update(valid=True, lat=10.0, lon=20.0, pos_at=_clock[0], rx=_clock[0])
_cf0 = hl._close_file
hl._close_file = lambda: True
hl.log["last_sample"] = _clock[0]
hl.log_tick()
_gap = _Rt.trip
hl._close_file = _cf0
hl.log_reset()
_rfresh()
hl.st.update(valid=False, pos_at=None)
check("ROUTE-TRIP: more than NEW_FILE_GAP between lines, a stay's end and the fix gap of a "
      "new file each start a trip: no km across",
      _same[1] == 1 and abs(_same[0] - 20000) < 5 and _new[1] == 2 and _new[0] == _same[0]
      and _leave == "20260615T113000Z" and _gap == "20260615T113000Z",
      repr((_same, _new, _leave, _gap)))

# the file: a reboot's little loss, and anything wrong with it ignored
_rfresh()
for _tr in _trips:
    _rfeed(*_tr)
_rview(_DAY + 7000)
_n0, _c0, _km0, _st0a = _Rt.n, _Rt.cm, _Rt.km, _Rt.step
_p0 = list(_Rt.pts)[:2 * _n0]
with open(hl.ROUTE_FILE, "rb") as f:
    _good = f.read()
_size_ok = len(_good) == 24 + 8 * _n0 + 4 and len(_good) <= 24 + 8 * hl.ROUTE_CAP + 4


def _rboot(data, clock=None):
    """A boot with this file; clock: the first fix's seconds."""
    hl._Rt.pts = None
    _rfresh()
    if data is not None:
        with open(hl.ROUTE_FILE, "wb") as f:
            f.write(data)
        try:
            hl._rt_load()
        except (ValueError, TypeError):
            return "raised"
    hl.log["clock"] = None
    if clock is not None:
        _rview(clock)
    return _Rt.n, _Rt.cm, _Rt.km


_same_day = _rboot(_good, _DAY + 7100)
_pts_back = list(_Rt.pts)[:2 * _n0] == _p0 and _Rt.step == _st0a and _Rt.view[1] == _n0
_rfeed(_rat(0, 0, _DAY + 7150))
_goes_on = _Rt.cm == _c0 and sum(_Rt.pts[2 * i] & 1 for i in range(_Rt.n)) == 3   # (long off: a new trip)
_rboot(_good, _DAY + 7100)
_last_t = max(r[2] for t in _ROUTE_DAY for r in t)
_rfeed(_rat(_ROUTE_DAY[-1][-1][0] / 10 + 100, _ROUTE_DAY[-1][-1][1] / 10, _DAY + _last_t + 60))
_short = abs(_Rt.cm - _c0 - 10000) < 15 and sum(_Rt.pts[2 * i] & 1 for i in range(_Rt.n)) == 2
_rboot(_good)
_wait = (_Rt.waiting, _Rt.view, _Rt.km)
_rview()
_wait2 = (_Rt.waiting, _Rt.view and _Rt.view[1], _Rt.km, _Rt.cm)
_old_day = _rboot(_good, _DAY + 86400)
_rboot(_good)
_rview()
_before = (_Rt.waiting, _Rt.view[1], _Rt.km, _Rt.view[3])
with open(hl.ROUTE_FILE, "rb") as _f:
    _kept = _f.read() == _good           # (a look without a fix writes nothing)
_rview(_DAY + 86400)
_after = (_Rt.waiting, _Rt.view[1], _Rt.km, _Rt.view[3] != _before[3])
_rboot(_good)
_rview()
_rview(_DAY + 7100)
_same = (_Rt.waiting, _Rt.view[1], _Rt.km, _Rt.cm)
_rboot(None)
_rview()
_no_file = (_Rt.view, _Rt.km)
check("ROUTE-BEFORE-FIX: a saved route shows at once without a fix, and writes nothing; the "
      "first fix keeps it the same day and empties it, redrawn, another day; no file: no view",
      _wait2 == (True, _n0, _km0, _c0) and _before[:3] == (True, _n0, _km0) and _kept
      and _after == (False, 0, "0,0 km", True) and _same == (False, _n0, _km0, _c0)
      and _no_file == (None, ""),
      repr((_wait2, _before, _kept, _after, _same, _no_file)))


# NTP: the clock and the route's day before the first fix
class _NtpMod:
    SOCK_DGRAM = 2
    reply = None
    echo = True                          # the answer's originate time: the request's
    addr = [(2, 2, 17, "", ("192.0.2.1", 123))]
    asked, closed, sent, wait = [], [], [], []

    @staticmethod
    def getaddrinfo(host, port, fam, typ):
        _NtpMod.asked.append((host, port, typ))
        if isinstance(_NtpMod.addr, Exception):
            raise _NtpMod.addr
        return _NtpMod.addr

    class socket:
        def __init__(self, fam, typ, proto):
            pass

        def settimeout(self, t):
            _NtpMod.wait.append(t)

        def sendto(self, b, a):
            _NtpMod.sent.append(bytes(b))

        def recv(self, n):
            r = _NtpMod.reply
            if isinstance(r, Exception):
                raise r
            if _NtpMod.echo and len(r) >= 32:
                r = r[:24] + _NtpMod.sent[-1][40:48] + r[32:]
            return r

        def close(self):
            _NtpMod.closed.append(1)


def _nreply(s, first=0x24, stratum=2, size=48):
    b = bytearray(48)
    b[0], b[1] = first, stratum
    b[40:44] = ((s + hl._NTP_2020) & 0xFFFFFFFF).to_bytes(4, "big")
    return bytes(b[:size])


def _nask(reply, addr=None):
    _NtpMod.reply = reply
    _NtpMod.addr = [(2, 2, 17, "", ("192.0.2.1", 123))] if addr is None else addr
    return hl._ntp_ask()


_n_sock, _n_build, _n_srv, _n_wifi = hl.socket, hl.FW_BUILD, hl.NTP_SERVER, hl.wifi["state"]
hl.socket = _NtpMod
hl.FW_BUILD = hl.EPOCH_DAYS * 86400 + _DAY - 86400 * 30
_NtpMod.closed[:] = []
_n_good = _nask(_nreply(_DAY))
_n_query = _NtpMod.sent[-1][0] == 0x23 and len(_NtpMod.sent[-1]) == 48
_n_bad = [_nask(x) for x in (b"", _nreply(_DAY, size=47), _nreply(_DAY, first=0x23),
                             _nreply(_DAY, first=0xE4), _nreply(_DAY, stratum=0),
                             _nreply(_DAY, stratum=16), _nreply(_DAY - 86400 * 31),
                             _nreply(-hl._NTP_2020), OSError(110))]
_n_closed = len(_NtpMod.closed) == 10
_n_nodns = [_nask(_nreply(_DAY), OSError(-2)), _nask(_nreply(_DAY), []) ]
check("NTP-ASK: a good answer in seconds since 2020; short, not a server's, unsynchronised, "
      "stratum 0 or 16, before this build, an empty time, a timeout or no DNS: None; "
      "the socket always closed",
      _n_good == _DAY and _n_query and _n_bad == [None] * 9 and _n_closed
      and _n_nodns == [None, None], repr((_n_good, _n_bad, _n_closed, _n_nodns)))
_n_tx = _NtpMod.sent[-1][40:48] == _clock[0].to_bytes(4, "big") + bytes(4)
_NtpMod.echo = False
_n_stray = _nask(_nreply(_DAY))
_NtpMod.echo = True
_n_era_s = (hl._days(2037, 1, 1) - hl.EPOCH_DAYS) * 86400 + 3600
_n_fw = hl.FW_BUILD
hl.FW_BUILD = hl._days(2033, 1, 1) * 86400      # (a build of 2033: NTP_ERA reaches 2037)
_n_era = _nask(_nreply(_n_era_s))
hl.FW_BUILD = _n_fw
check("NTP-ASK-ECHO: the request carries the ticks as its transmit time; an answer whose "
      "originate time is not that (a stray or late datagram) is None; a time past the "
      "NTP era of 2036 reads as the next era (for a build of 2033, RG-4 has the rest)",
      _n_tx and _n_stray is None and _n_era == _n_era_s, repr((_n_tx, _n_stray, _n_era, _n_era_s)))

_n0a = _n0
hl._ntp[:] = [None, None, hl.NTP_RETRY]
hl.log["clock"] = None
_NtpMod.asked[:] = []
_NtpMod.reply, _NtpMod.addr = OSError(110), [(2, 2, 17, "", ("192.0.2.1", 123))]
hl.wifi["state"] = "sleep"
hl.ntp_tick(_clock[0])
_n_offline = len(_NtpMod.asked)
hl.wifi["state"] = "connected"
hl.NTP_SERVER = None
hl.ntp_tick(_clock[0])
_n_off = len(_NtpMod.asked)
hl.NTP_SERVER = "ntp.test"
hl.ntp_tick(_clock[0])
hl.ntp_tick(_clock[0])
_n_once = len(_NtpMod.asked)
_n_w1 = hl._ntp[2]
_clock[0] += hl.NTP_RETRY * 2
hl.ntp_tick(_clock[0])
_n_w2 = hl._ntp[2]
for _i in range(12):
    _clock[0] += hl.NTP_RETRY_MAX
    hl.ntp_tick(_clock[0])
_n_cap = hl._ntp[2]
_NtpMod.reply = _nreply(_DAY)
_clock[0] += hl.NTP_RETRY_MAX
_n_lines = len(hl.log["batch"])
_n_diag = []
_n_dw = hl._diag_write
hl._diag_write = _n_diag.append
hl.ntp_tick(_clock[0])
hl._diag_write = _n_dw
_n_ok = hl._ntp[0] == (_DAY, _clock[0]) and hl._ntp[2] == hl.NTP_AGAIN
_n_asked = len(_NtpMod.asked)
hl.ntp_tick(_clock[0] + hl.NTP_AGAIN - 1)
_n_held = len(_NtpMod.asked) == _n_asked
hl.log["clock"] = (_DAY, _clock[0])
_n_url = hl.up["url"]
hl.up["url"] = "http://d.example"        # (https asks on: RN2-5)
hl.ntp_tick(_clock[0] + hl.NTP_AGAIN * 2)
hl.up["url"] = _n_url
_n_fixed = len(_NtpMod.asked) == _n_asked
hl.log["clock"] = None
hl.ntp_tick(_clock[0] + hl.NTP_AGAIN)
_n_again = len(_NtpMod.asked) == _n_asked + 1
check("NTP-TICK: only connected, with a server and before a fix told the time; a failed try "
      "waits, doubling to the cap; an answer lasts NTP_AGAIN, says one diag line and "
      "writes no track line",
      _n_offline == 0 and _n_off == 0 and _n_once == 1 and _n_w1 == 2 * hl.NTP_RETRY
      and _n_w2 == 4 * hl.NTP_RETRY and _n_cap == hl.NTP_RETRY_MAX and _n_ok and _n_held
      and _n_fixed and _n_again and len(hl.log["batch"]) == _n_lines
      and _n_diag == ["clock ntp " + hl._stamp_s(_DAY)[0]],
      repr((_n_offline, _n_off, _n_once, _n_w1, _n_w2, _n_cap, _n_ok, _n_held, _n_fixed, _n_again, _n_diag)))

# (_DAY: 10:00 UTC on 2026-06-15, 12:00 in summer time)
_clock[0] += 1000
hl.st["rx"] = _clock[0]

hl._ntp[0] = (_DAY, _clock[0])
_h_summer = _ls(None, None)
hl._ntp[0] = (_DAY - 86400 * 150, _clock[0])
_h_winter = _ls(None, None)
_h_gps = _ls("2026-11-04", "10:42:10")
hl.st["rx"] = _clock[0] - hl.NO_DATA
hl._ntp[0] = (_DAY, _clock[0])
_h_silent = _ls("2026-11-04", "10:42:10")
hl.st["rx"] = _clock[0]
hl._ntp[0] = (_DAY, _clock[0] - 60000)
_h_runs = _ls(None, None)
_clock_old = _clock[0]
hl._ntp[0] = None
_h_none = _ls(None, None)
check("NTP-CLOCK: before a fix the clock is NTP's in Hungarian time, summer and winter, and "
      "runs on; a fix wins; without either --:--",
      _h_summer == "12:00" and _h_winter == "11:00" and _h_gps == "11:42" and _h_silent == "12:00"
      and _h_runs == "12:01" and _h_none == "--:--",
      repr((_h_summer, _h_winter, _h_gps, _h_silent, _h_runs, _h_none)))
hl.st.update(date=None, time=None)

_rboot(_good)
_rview()
hl._ntp[0] = (_DAY + 7100, _clock[0])
_rview()
_r_same = (_Rt.waiting, _Rt.view[1], _Rt.km, _Rt.cm)
_rboot(_good)
_rview()
hl._ntp[0] = (_DAY + 86400, _clock[0])
_rview()
_r_next = (_Rt.waiting, _Rt.view[1], _Rt.km)
_rboot(None)
hl._ntp[0] = (_DAY, _clock[0])
_rview()
_r_nofile = (_Rt.view[1], _Rt.km)
_rboot(_good)
hl._ntp[0] = (_DAY + 86400, _clock[0])
hl.log["clock"] = (_DAY + 7100, _clock[0])
_rview()
_r_fixwins = (_Rt.view[1], _Rt.km, _Rt.cm)
hl.log["clock"] = None
hl._ntp[:] = [None, None, hl.NTP_RETRY]
check("NTP-ROUTE: NTP tells the route's day before a fix: the same day kept, another emptied, "
      "no file an empty day; a fix's clock wins over it",
      _r_same == (False, _n0a, _km0, _c0) and _r_next == (False, 0, "0,0 km")
      and _r_nofile == (0, "0,0 km") and _r_fixwins == (_n0a, _km0, _c0),
      repr((_r_same, _r_next, _r_nofile, _r_fixwins)))

_e_srv = []
for _e in ({"NTP_SERVER": "ntp.lan # the router"}, {"NTP_SERVER": ""}, {}):
    hl.NTP_SERVER = "kept"
    hl._ntp_env(_e)
    _e_srv.append(hl.NTP_SERVER)
check("NTP-ENV: NTP_SERVER names the server, empty turns it off, absent keeps the default",
      _e_srv == ["ntp.lan", None, "kept"], repr(_e_srv))
hl.socket, hl.FW_BUILD, hl.NTP_SERVER, hl.wifi["state"] = _n_sock, _n_build, _n_srv, _n_wifi
_rboot(None)
_flip = bytearray(_good)
_flip[40] ^= 1
_bad_n = bytearray(_good)
_bad_n[20] = 200
_badm = bytearray(_good)
_badm[0] ^= 1
_torn = [_good[:k] for k in (0, 1, 4, 23, 24, 27, 28, 40, len(_good) - 5, len(_good) - 4, len(_good) - 1)]
_junk = [bytes(_flip), bytes(_bad_n), bytes(_badm), b"\x00" * len(_good), b"\xff" * 700, _good + b"x"]
_ign = [_rboot(d, _DAY + 7100) for d in _torn + _junk]
_none_file = _rboot(None, _DAY + 7100)
check("ROUTE-FILE: saved and read back the same day whole; another day's, a torn or garbled "
      "one, or none: an empty day once a fix tells the date, nothing raised",
      _size_ok and _same_day == (_n0, _c0, _km0) and _pts_back and _wait == (True, None, "")
      and _goes_on and _short
      and _old_day == (0, 0, "0,0 km")
      and all(i == (0, 0, "0,0 km") for i in _ign) and _none_file == (0, 0, "0,0 km"),
      repr((_size_ok, _same_day, _pts_back, _wait, _wait2, _old_day, _ign, _none_file)))
_tmp_left = _rfresh() or os.path.exists(hl.ROUTE_FILE + ".tmp")
_rn0 = os.rename
_renames = []


def _rn_count(a, b):
    _renames.append((a, b))
    return _rn0(a, b)


os.rename = _rn_count
_rt_s = _DAY
_rx_ = 0
hl.log["clock"] = (_rt_s, _clock[0])
for _m in range(60):                     # a drive of an hour: a batch a minute, 6 looks
    _rfeed(*[_rat(_rx_ + _i * 10, 0, _rt_s + _m * 60 + _i) for _i in range(60)])
    _rx_ += 600
    for _i in range(6):
        _clock[0] += hl.ROUTE_DRAW
        hl.log["clock"] = (_rt_s + _m * 60 + _i * 10, _clock[0])
        hl._rt_tick(_clock[0])
_w_drive = len(_renames)
_via_tmp = _renames and all(a == hl.ROUTE_FILE + ".tmp" and b == hl.ROUTE_FILE for a, b in _renames)
_clock[0] += hl.ROUTE_DRAW
hl.log["clock"] = (_rt_s + 3599, _clock[0])
hl._rt_tick(_clock[0])
_renames[:] = []
hl.log["still"]["active"] = True
_rfeed(_rat(_rx_, 0, _rt_s + 3600))
_w_stay0 = 0
for _i in range(360):                    # an hour's stay: one write at its start
    _clock[0] += hl.ROUTE_DRAW
    hl.log["clock"] = (_rt_s + 3600 + _i * 10, _clock[0])
    hl._rt_tick(_clock[0])
    if not _i:
        _w_stay0 = len(_renames)
_w_stay = len(_renames)
_renames[:] = []
for _i in range(6 * 60 * 24):            # standing a day with nothing new: none
    _clock[0] += hl.ROUTE_DRAW
    hl.log["clock"] = (_rt_s + 3600 + _i * 10, _clock[0])
    hl._rt_tick(_clock[0])
_w_idle = len(_renames)
hl.log["still"]["active"] = False
_open0 = builtins.open


def _ro_open(path, mode="r", *a, **k):
    if str(path).endswith(".tmp"):
        raise OSError(28)
    return _open0(path, mode, *a, **k)


_renames[:] = []
_rfeed(_rat(_rx_ + 100, 0, _rt_s + 3700))
hl.log["still"]["active"] = True
with open(hl.ROUTE_FILE, "rb") as f:
    _keep_old = f.read()
builtins.open = _ro_open
_bad_tick = 0
_s_fail = True
try:
    for _i in range(30):                 # 5 minutes of a stay on a failing flash
        _clock[0] += hl.ROUTE_DRAW
        hl.log["clock"] = (_rt_s + 3700 + _i * 10, _clock[0])
        try:
            hl._tft_state(_clock[0])
        except Exception:
            _s_fail = False
        _bad_tick += hl._tft_t[4]
finally:
    builtins.open = _open0
os.rename = _rn0
hl.log["still"]["active"] = False
_low_ok = hl._tft is _ft
with open(hl.ROUTE_FILE, "rb") as f:
    _old_safe = f.read() == _keep_old
check("ROUTE-WRITES: at most one write per ROUTE_SAVE on a drive, one at a stay's start, none "
      "while nothing changed, by rename from a .tmp; a failing flash only retries that seldom "
      "and does not turn the display dark",
      _w_drive <= 13 and _w_stay0 == 1 and _w_stay == 1 and _w_idle == 0 and _s_fail and _low_ok
      and _bad_tick <= 2 and not _tmp_left and _via_tmp and _old_safe,
      repr((_w_drive, _w_stay0, _w_stay, _w_idle, _s_fail, _low_ok, _bad_tick, _via_tmp, _old_safe)))

# the km's text, and its room
_kt = []
for _cm in (0, 859800, 994999, 995000, 9994999, 9995000, 99949999):
    _rfresh()
    _Rt.cm = _cm
    hl._rt_project()
    _kt.append(_Rt.km)
_kw = max(_ft.width(_ft._SMALL, k) for k in _kt + ["99,9 km", "999 km"])
_show(km="999 km", level=3)
_k1 = _rect(_ft.KM_X, 31, 140, 42) - {_PAPER_C}
_k_col = _rect(_ft.KM_X + _kw, 31, 140, 42) == {_PAPER_C}
_show(km="99,9 km", level=3)
_k2 = _rect(_ft.KM_X, 31, 140, 42) - {_PAPER_C}
_k2_col = _rect(_ft.KM_X + _ft.width(_ft._SMALL, "99,9 km"), 31, 140, 42) == {_PAPER_C}
_kcomma = max(y for x in range(92, 139) for y in range(30, 47) if _px(x, y) != _PAPER_C)
_show(km="999 km", level=3)
_kbase = max(y for x in range(92, 139) for y in range(30, 47) if _px(x, y) != _PAPER_C)
_kleft = min(x for x in range(90, 139) for y in range(30, 47) if _px(x, y) != _PAPER_C)
check("ROUTE-KM: a decimal comma under 100, whole km above, rounded; at x=92 on baseline 42 "
      "in ink, and the widest texts end short of the bars",
      _kt == ["0,0 km", "8,6 km", "9,9 km", "10,0 km", "99,9 km", "100 km", "999 km"]
      and _kw < 140 - _ft.KM_X - 1 and _k_col and _k2_col and _k1 <= _pal16(_ft.INK, _ft.PAPER)
      and _kbase == 41 and 92 <= _kleft <= 93 and _k2 <= _pal16(_ft.INK, _ft.PAPER)
      and _kcomma < 47, repr((_kt, _kw, _k_col, _k2_col, _kbase, _kcomma, _kleft)))

# the route on the panel
_GRID_DOTS = [(4 + i * 8, 4 + j * 8) for j in range(7) for i in range(11)]
_show(route=None, km="", level=None)
_grid_only = all(_px(x, y) == _ft.DIM and _px(x + 1, y + 1) == _ft.DIM for x, y in _GRID_DOTS)
_paper_else = sum(1 for y in range(0, 63) for x in range(0, 92) if _px(x, y) != _PAPER_C) == 4 * len(_GRID_DOTS)
_rfresh()
_empty_v = _rview(_DAY)
_show(route=_empty_v, km=_Rt.km)
_note = [(x, y) for y in range(0, 63) for x in range(0, 92)
         if _px(x, y) not in (_PAPER_C, _ft.DIM)]
_note_ok = (_note and _empty_v[1] == 0 and min(x for x, y in _note) >= 4 and max(x for x, y in _note) <= 88
            and abs(min(x for x, y in _note) + max(x for x, y in _note) - 92) <= 3)
check("ROUTE-EMPTY: no day known yet: the grid alone, 2x2 on the 8 px pitch, nothing else in "
      "the field; a known empty day: the grid and the short note, centred; no \"--\" anywhere",
      _grid_only and _paper_else and _note_ok and "-" not in _ff.SMALL[0],
      repr((_grid_only, _paper_else, _note_ok)))


def _on_row(y, x0, x1, c):
    return all(_px(x, y) == c for x in range(x0, x1))


# one trip east, a gap, a second one: y is the middle row: 1000 m wide: 12 m a pixel
_rfresh()
_rfeed(_rat(0, 0, _DAY), _rat(400, 0, _DAY + 60))
hl._rt_trip(_DAY + 120)
_rfeed(_rat(600, 0, _DAY + 120), _rat(830, 0, _DAY + 180))
_v = _rview(_DAY + 200)
_show(route=_v, km=_Rt.km)
_y = 4 + (_v[0][1] & 127)
_ink_part = _on_row(_y, 8, 44, _INK_C) and _on_row(_y + 1, 8, 44, _INK_C)
_no_chord = _rect(46, _y, 62, _y + 2) <= {_PAPER_C, _ft.DIM}
_red_part = _on_row(_y, 66, 84, _RED_C) and _on_row(_y + 1, 66, 84, _RED_C)
_thin = _px(20, _y - 1) in (_PAPER_C, _ft.DIM) and _px(20, _y + 2) in (_PAPER_C, _ft.DIM)
_start = (_px(2, _y - 2), _px(3, _y - 2), _px(2, _y - 1), _px(7, _y + 3), _px(6, _y + 3), _px(7, _y + 2))
_end_x = 4 + _v[0][2 * (_v[1] - 1)]
_end = (_end_x, _px(_end_x - 2, _y - 2), _px(_end_x - 1, _y - 2), _px(_end_x - 2, _y - 1), _px(_end_x + 3, _y + 3),
        _px(_end_x + 2, _y + 3), _px(_end_x + 3, _y + 2))
_field_ok = _rect(0, 0, 2, 2) == {_PAPER_C}
check("ROUTE-DRAW: north up, 2x2 dots: the earlier trip ink, the latest red, no line between "
      "them; 6x6 round dots: ink at the day's start, red at the current position",
      _ink_part and _no_chord and _red_part and _thin
      and _start == (_PAPER_C, _INK_C, _INK_C, _PAPER_C, _INK_C, _INK_C)
      and _end == (87, _ft.DIM, _RED_C, _RED_C, _PAPER_C, _RED_C, _RED_C)     # (a grid dot's in that corner)
      and _field_ok and _v[2] == 2, repr((_ink_part, _no_chord, _red_part, _thin, _start, _end, _v[:1] and list(_v[0][:8]), _v[1:3])))
# north is up, in a band's split too: a line across all the bands
_rfresh()
_rfeed(_rat(0, 0, _DAY), _rat(0, 800, _DAY + 60))
_v = _rview(_DAY + 70)
_show(route=_v, km=_Rt.km)
_x = 4 + _v[0][0]
_col = [y for y in range(0, 63) if _px(_x + 3, y) == _RED_C or _px(_x, y) == _RED_C]
_ys = (_v[0][1] & 127, _v[0][3] & 127)
_full = all(_px(_x, y) == _RED_C for y in range(10, 50))
_start_low = _px(_x, 4 + _ys[0] + 3) == _INK_C and _px(_x, 4 + _ys[1]) == _RED_C
check("ROUTE-NORTH: a walk due north runs from the ink dot at the bottom to the red one at "
      "the top, unbroken across the bands",
      _ys == (51, 0) and _full and _start_low, repr((_ys, _full, _start_low)))
_rfresh()
_rfeed(_rat(0, 0, _DAY), _rat(100, 0, _DAY + 60))
_v = _rview(_DAY + 70)
_wid = _v[0][2] - _v[0][0]
_rfresh()
_rfeed(_rat(0, 0, _DAY), _rat(100, 0, _DAY + 30), _rat(100, 100, _DAY + 60), _rat(0, 100, _DAY + 90))
_v2 = _rview(_DAY + 100)
_h2 = (_v2[0][3] & 127) - (_v2[0][5] & 127)
check("ROUTE-SPAN: 500 m across at the least: 100 m is 16 or 17 pixels, not the whole field, "
      "and a one-point day stays in the middle",
      16 <= _wid <= 17 and abs(_v[0][0] + _v[0][2] - hl.ROUTE_W) <= 1 and 16 <= abs(_h2) <= 17,
      repr((_wid, list(_v[0][:4]), _h2)))

# no fix: the route stays, what the rules say about a fix still holds
_rfresh()
for _tr in _trips:
    _rfeed(*_tr)
_v = _rview(_DAY + 7000)
_show(route=_v, km=_Rt.km, level=None)
_before_px = [_px(x, y) for y in range(0, 63) for x in range(0, 92)]
_show(route=_v, km=_Rt.km, fix=False, used=0, clock="--:--", quality=0, level=None)
_after_px = [_px(x, y) for y in range(0, 63) for x in range(0, 92)]
_nofix = _before_px == _after_px and _sat_ok(0, _RED_C) and _px(0, 64) == _RED_C and _px(30, 66) == _RED_C
_inks = sum(1 for p in _after_px if p == _INK_C), sum(1 for p in _after_px if p == _RED_C)
check("ROUTE-NOFIX: the route and the km stay as they were, the count goes red and the tile "
      "says nincs; both trips on screen, in their colours", _nofix and _inks[0] > 50 and _inks[1] > 50,
      repr((_nofix, _inks)))

# the current position: the line just recorded, before flush; not during a stay
_rfresh()
hl.log_reset()
_rfeed(_rat(0, 0, _DAY), _rat(400, 0, _DAY + 60))
_rv0 = _rview(_DAY + 70)
_ver0 = _rv0[3]
_rv1 = _rview(_DAY + 71)                 # nothing changed: the very same view
_same_view = _rv1 is _rv0
hl.log["prev"] = (10.0, 20.0 + 800 / M_E6 / 1e6 / math.cos(math.radians(10.0)))
hl.st.update(valid=True, date="2026-06-15", time="10:01:30", lat=10.0, lon=20.1, pos_at=_clock[0] + hl.ROUTE_DRAW,
             rx=_clock[0] + hl.ROUTE_DRAW)
_rv2 = _rview(_DAY + 72)
_tail_on = _rv2[1] == _rv0[1] + 1 and _rv2[3] > _ver0 and _Rt.cm < 45000
_tail_x = _rv2[0][2 * (_rv2[1] - 1)]
_c_tail = _rv2[1]
_tick_before = _Rt.drawn
_clock[0] += hl.ROUTE_DRAW - 1
hl.log["prev"] = (10.0, 20.0)
hl._rt_tick(_clock[0])
_gated = _Rt.view is _rv2 and _Rt.drawn == _tick_before
hl.st["pos_at"] = _clock[0] + hl.ROUTE_DRAW
hl.log["still"]["active"] = True
_rv3 = _rview(_DAY + 73)
_in_stay = _rv3[1] == _rv0[1]
hl.log["still"]["active"] = False
hl.log_reset()
hl.st.update(valid=False, pos_at=None)
check("ROUTE-TICK: an unchanged route keeps its very view (no redraw); the position just "
      "recorded leads the route without adding km; looked at only every ROUTE_DRAW; a stay's "
      "drift does not move it",
      _same_view and _tail_on and _tail_x == hl.ROUTE_W and _gated and _in_stay,
      repr((_same_view, _tail_on, _tail_x, _gated, _in_stay, _c_tail)))

hl._Rt.pts = None                        # (as on a board without the display)
hl.ROUTE_FILE = "/route.bin"
_show()
_ft.set_state(None, "", True, 22, "11:42", 3, 3, "kész", False)
_ft.frame()
_c6 = gl._Rt.pts is None and gl._rt_feed(["x"], 1) is None

# the fonts: only the rows from the first to the last with ink in any glyph, BASE counted from the first kept: put
# back, they are the bytes the tool wrote before the crop, and a string stays on
# its rows. A new font or new characters on purpose: new values here
_font_was = {  # uncropped: HEIGHT, BASE, sha256 of DATA (first 32)
    "CLOCK": (20, 15, "b5fed7c4675f1ab25a14a5ca62adc5a7"),
    "SATS": (15, 11, "0edb1e7fe3a52847b2ace6589dd4edb0"),
    "UP": (19, 14, "d343575aba74d600ad6f87e6ae21d612"),
    "WORD": (16, 12, "2c16dd844e8e8f8c15042cbd7159b776"),
    "SMALL": (13, 10, "39917aa852d8543e8ffbad6189b77108"),
}
_font_rows = (  # the rows a string's ink takes, against its baseline
    ("CLOCK", "11:42", (-11, -1)), ("CLOCK", "--:--", (-8, -1)), ("SATS", "22", (-8, -1)),
    ("UP", "kész", (-10, -1)), ("UP", "12k", (-10, -1)), ("WORD", "erős", (-9, -1)),
    ("WORD", "gyenge", (-6, 1)), ("WORD", "közepes", (-8, 1)), ("WORD", "nincs", (-8, -1)),
    ("SMALL", "<1 óra", (-7, -1)), ("SMALL", "7 nap", (-7, 1)), ("SMALL", "8,6 km", (-7, 2)),
    ("SMALL", "100%", (-7, -1)),
)


def _font_glyphs(f):
    """[(the glyph's rows, each (w + 1) // 2 bytes)]: None when OFFS or the
    size of DATA do not fit HEIGHT."""
    chars, widths, offs, data, h = f[:5]
    if len(widths) != len(chars) or len(offs) != 2 * len(chars):
        return None
    out, o = [], 0
    for i, w in enumerate(widths):
        n = (w + 1) // 2
        if offs[i * 2] | offs[i * 2 + 1] << 8 != o:
            return None
        out.append([data[o + r * n:o + (r + 1) * n] for r in range(h)])
        o += n * h
    return out if o == len(data) else None


def _font_pad(f, h0, b0):
    """f with the blank rows back as it had h0 rows, its baseline at b0."""
    top = b0 - f[5]
    pad, offs = bytearray(), bytearray()
    for i, g in enumerate(_font_glyphs(f) or ()):
        n = (f[1][i] + 1) // 2
        offs += len(pad).to_bytes(2, "little")
        pad += bytes(n * max(0, top)) + b"".join(g) + bytes(n * max(0, h0 - top - f[4]))
    return (f[0], f[1], bytes(offs), bytes(pad), h0, b0)


_fnames = ("CLOCK", "SATS", "UP", "WORD", "SMALL")
_ffonts = {n: getattr(_ff, n) for n in _fnames}
_fg = {n: _font_glyphs(f) for n, f in _ffonts.items()}
_ftight = {n: bool(_fg[n]) and all(any(any(g[r]) for g in _fg[n]) for r in (0, _ffonts[n][4] - 1))
           for n in _fnames}
check("TFT-FONT-CROP: no font keeps a row without ink above or below; OFFS and the size "
      "of DATA fit HEIGHT", all(_fg.values()) and all(_ftight.values()), repr(_ftight))
_fsha = {n: hashlib.sha256(_font_pad(_ffonts[n], *_font_was[n][:2])[3]).hexdigest()[:32] for n in _fnames}
check("TFT-FONT-BASE: BASE and HEIGHT say where the crop was: the blank rows back on that "
      "side give the uncropped bytes",
      all(_fsha[n] == _font_was[n][2] and 0 <= _font_was[n][1] - _ffonts[n][5]
          and _font_was[n][1] - _ffonts[n][5] + _ffonts[n][4] <= _font_was[n][0] for n in _fnames),
      repr((_fsha, [_ffonts[n][4:] for n in _fnames])))
_fpair = dict(zip(_fnames, (_ft._CLOCK, _ft._SATS, _ft._UP, _ft._WORD, _ft._SMALL)))
_frows = []
_band0, _ft.BAND_H = _ft.BAND_H, 64      # (one tall band: the string is on it)
for _n, _s, _want in _font_rows:
    # (the band as gps_tft has it: a string blitted at a row, read back)
    _fb = _FakeFramebuf.FrameBuffer(bytearray(160 * 64 * 2), 160, 64, _FakeFramebuf.RGB565)
    _fb.fill(_PAPER_C)
    _ft._text(_fb, _fpair[_n], _s, 2, 40, _ft.INK)
    _ink = [y - 40 for y in range(64) if any(_fb.pixel(x, y) != _PAPER_C for x in range(160))]
    _frows.append((_n, _s, (_ink[0], _ink[-1]) if _ink else None))
_ft.BAND_H = _band0
check("TFT-FONT-ROWS: a string stays on its rows against the baseline",
      _frows == list(_font_rows), repr(_frows))
_fv = (bytearray(b"\x00\x80\x50\x19"), 2, 0, 1)
_fstates = ((_fv, "8,6 km", True, 22, "11:42", None, 3, "1284", True),
            (None, "", False, 0, "--:--", 0, 0, "kész", False),
            (_fv, "99,9 km", True, 9, "09:05", 2, 2, " ", False, (70, "44 óra")),
            (_fv, "999 km", True, 40, "23:58", 3, 1, "12k", True, (96, "<1 óra")),
            ((_fv[0], 0, 0, 2), "0,0 km", True, 1, "06:37", 1, 1, "kész", False, (100, "tele")))


def _font_panels():
    out = []
    for st in _fstates:
        _ft._D.top = _ft._D.low = None
        _ft.set_state(*st)
        _ft.frame()
        out.append(dict(_TftSpi.panel))
    return out


_fp1 = _font_panels()
_fsaved = _ft._CLOCK, _ft._SATS, _ft._UP, _ft._WORD, _ft._SMALL
_ft._CLOCK, _ft._SATS, _ft._UP, _ft._WORD, _ft._SMALL = [
    (_font_pad(_ffonts[n], *_font_was[n][:2]), memoryview(_font_pad(_ffonts[n], *_font_was[n][:2])[3]))
    for n in _fnames]
_fp2 = _font_panels()
_ft._CLOCK, _ft._SATS, _ft._UP, _ft._WORD, _ft._SMALL = _fsaved
check("TFT-FONT-PANEL: the panel pixel for pixel as with the blank rows back: the route's "
      "note, km, satellites, clock, WiFi, quality words, upload and the gauge's texts",
      _fp1 == _fp2 and len(_fp1[0]) == 160 * 80,
      repr([sum(1 for k in a if a[k] != b.get(k)) for a, b in zip(_fp1, _fp2)]))
_show()
# the tool's crop, on a font small enough to read (importing it writes nothing)
_mf_path = os.path.join(ROOT, "tools", "make_tft_font.py")
with open(_mf_path, encoding="utf-8") as f:
    _mf_src = f.read()
_mf = {}
if "def crop(" in _mf_src and '__name__ == "__main__"' in _mf_src:
    exec(compile(_mf_src, _mf_path, "exec"), {"__name__": "make_tft_font"}, _mf)
_crop = _mf.get("crop")
_row0 = b"\x00"
_tiny = (_row0 + b"\x10" + b"\x20" + _row0                 # width 2: a byte a row
         + _row0 * 2 + b"\x00\x30" + b"\x40\x00" + _row0 * 2)  # width 3: two
try:
    _crop_err = _crop and _crop(b"\x02", b"\x00" * 3, 2, 1)
except ValueError:
    _crop_err = "refused"
check("TFT-FONT-TOOL: crop keeps the rows from the first to the last with ink in any "
      "glyph, a blank row between them, BASE from the first kept; all blank: one row; "
      "data of the wrong size refused; importing the tool runs nothing",
      _crop is not None
      and _crop(b"\x02\x03", _tiny + b"", 4, 3) == (b"\x00\x00\x02\x00", b"\x10\x20\x00\x30\x40\x00", 2, 2)
      and _crop(b"\x02", b"\x00\x10\x00\x20", 4, 2) == (b"\x00\x00", b"\x10\x00\x20", 3, 1)
      and _crop(b"\x02", b"\x00" * 3, 3, 2) == (b"\x00\x00", b"\x00", 1, 2)
      and _crop_err == "refused" and "main" in _mf, repr(_mf.keys()))
# what the tool writes is what is committed (needs Pillow and the Space Grotesk TTFs)
_mf_can = importlib.util.find_spec("PIL") is not None and bool(_mf) and os.path.exists(_mf["D"] % "Bold")
if not _mf_can:
    print("SKIP  TFT-FONT-REGEN: Pillow or the Space Grotesk TTFs are missing")
else:
    _mf_td = tempfile.mkdtemp(prefix="font-")
    _mf_r = subprocess.run([sys.executable, _mf_path], cwd=_mf_td, capture_output=True, text=True)
    with open(os.path.join(ROOT, "gps_tft_font.py"), "rb") as f:
        _mf_a = f.read()
    with open(os.path.join(_mf_td, "gps_tft_font.py"), "rb") as f:
        _mf_b = f.read()
    shutil.rmtree(_mf_td, True)
    check("TFT-FONT-REGEN: tools/make_tft_font.py writes the committed gps_tft_font.py",
          _mf_r.returncode == 0 and _mf_a == _mf_b, _mf_r.stderr[-200:])

# a panel gone wrong: dark, and each action counted when it runs
hl._tft = _ft
hl._tft_t[:] = [None, None, 0, 0, 0]
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
      "display dark for the boot (nothing frozen stays lit), said once, nothing leaves tft_tick",
      hl._tft is None and len(_dl) == 1 and _dl[0].startswith("bug tft ") and _dark,
      repr((hl._tft, _dl, _dark)))
_st0 = hl._tft_state
hl._tft = _ft
hl._tft_t[:] = [None, None, 0, 0, 0]
hl._tft_state = lambda now: (_ for _ in ()).throw(ValueError("state"))
hl._diag_write = lambda l: None
_pin_levels[:] = []
for _i in range(hl.TFT_MAX_ERR):
    _clock[0] += hl.TFT_STATE
    hl.tft_tick(_clock[0])               # a good frame between must not clear it
hl._tft_state, hl._diag_write = _st0, _dw
check("TFT-FAIL-STATE: values failing keep counting though frames succeed: no old state "
      "redrawn for ever", hl._tft is None
      and [p[:2] for p in _pin_levels if p[0] == 21][-1:] == [(21, 0)], repr(hl._tft))
_ft_seen = []
gl._tft = None
gl._rt_start, _rs0 = (lambda: _ft_seen.append("route")), gl._rt_start
sys.modules["gps_tft"] = types.SimpleNamespace(start=lambda *a: _ft_seen.append("start(%d, %d)" % a))
gl._tft_start()
gl._tft_start()                           # (started: not again)
sys.modules["gps_tft"] = _ft
gl._tft, gl._rt_start = None, _rs0
with open(os.path.join(ROOT, "gps_logger.py")) as f:
    _lsrc = f.read()
_runs = _lsrc[_lsrc.index("def run():"):]
_loops = _lsrc[_lsrc.index("def _loop("):]
check("TFT-WIRING: run() starts it, once, at TFT_LIGHT and the backlight pin (HELTEC_TFT_BL), "
      "after .env is read (TFT_LIGHT / TFT_IDLE: no glow at a level of 0) and before the first GPS "
      "setup; the loop ticks it",
      _ft_seen == ["start(%d, %d)" % (gl.TFT_LIGHT, gl.HELTEC_TFT_BL), "route"]
      and _runs.index("load_env(ENV_FILE)") < _runs.index("_tft_env(env)")
      < _runs.index("_tft_start()") < _runs.index("gps_setup()")
      and "tft_tick(now)" in _loops, repr(_ft_seen))

# --- the backlight: PWM, dark in a stay, full after the USER button (2026-11-08)
_Pwm.live.clear()
_pin_levels[:] = []
_ft._D.pwm = None
_ft.light(30)
_pw = _Pwm.live.get(21)
_l30 = _pw is not None and (_pw.freq, _pw.duty) == (2000, 30 * 65535 // 100)
_ft.light(100)
_l100 = _pw.duty == 65535
_ft.light(0)
_l0 = _pw.duty == 0
_ft.off()
_loff = _pw.duty == 0 and [p[:2] for p in _pin_levels if p[0] == 21][-1:] == [(21, 0)]
_Pwm.fail, _ft._D.pwm = True, None
_pin_levels[:] = []
_ft.light(30)
_ft.light(0)
_lpin = [p[:2] for p in _pin_levels if p[0] == 21]
_Pwm.fail, _ft._D.pwm = False, None
check("TFT-LIGHT: the backlight is a PWM at 2 kHz, pct per cent of the duty (30, 100, 0), off() "
      "is duty 0; without PWM it is on for any pct and off for 0, as before",
      _l30 and _l100 and _l0 and _loff and _lpin == [(21, True), (21, False)],
      repr((_l30, _l100, _l0, _loff, _lpin)))
_ft._D.btn = None
_pin_levels[:] = []
_b0 = _ft.pressed()                       # (the stub pin reads 1: up)
_bn = getattr(_ft._D.btn, "n", None)
_ft._D.btn = types.SimpleNamespace(value=lambda: 0)
_b1 = _ft.pressed()
_ft._D.btn = None
Pin.fail = (0,)
_b2 = _ft.pressed()
_bf = _ft._D.btn
Pin.fail = ()
_ft._D.btn = None
check("TFT-BUTTON: the USER button is GPIO0, pulled up, low is pressed; a pin that cannot be had "
      "reads as never pressed (once tried)",
      _b0 is False and _bn == 0 and _b1 is True and _b2 is False and _bf is False,
      repr((_b0, _bn, _b1, _b2, _bf)))


class _LightTft(object):
    """The display as the light logic sees it: the levels it was given."""
    def __init__(self):
        self.lv = []
    def light(self, p):
        self.lv.append(p)
    def pressed(self):
        return False


_dl, _dw = [], hl._diag_write
hl._diag_write = _dl.append
_t0 = hl._tft
hl._tft = _lt = _LightTft()
_g0 = (hl.TFT_LIGHT, hl.TFT_IDLE, hl.TFT_WAKE)
hl.TFT_LIGHT, hl.TFT_IDLE, hl.TFT_WAKE = 30, 300000, 60000
hl._tft_l[:] = [None, None, None]
_sl = hl.log["still"]
_sl["active"] = False
_T = 1000000
_seq = []
def _lv(dt):
    hl._tft_light(_T + dt)
    _seq.append(_lt.lv[-1])
_lv(0)                                    # day: the dim level
_sl["active"] = True
_lv(1000)                                 # a stay begins
_lv(1000 + 299000)                        # not yet 300 s old
_lv(1000 + 300000)                        # 300 s: dark
hl._tft_l[2] = _T + 400000                # the USER button
_lv(400000 + 59000)                       # full light
_lv(400000 + 60000)                       # 60 s: dark again (still in the stay)
_sl["active"] = False
_lv(500000)                               # moving: dim again
_stay = list(_seq)
_dln = list(_dl)
hl.TFT_IDLE = 0                           # never dark
_sl["active"] = True
_lv(600000)
_lv(600000 + 86400000 // 2)
_never = list(_seq[-2:])
hl.TFT_IDLE = 300000
hl.TFT_LIGHT = 0                          # the display dark by choice, the button still lights it
_sl["active"] = False
hl._tft_l[:] = [None, None, None]
_lt.lv[:] = []
hl._tft_light(_T)
hl._tft_l[2] = _T + 1000
hl._tft_light(_T + 2000)
hl._tft_light(_T + 1000 + 60000)
_zero = list(_lt.lv)
hl.TFT_LIGHT, hl.TFT_IDLE, hl.TFT_WAKE = _g0
hl._tft = _t0
_sl["active"] = False
check("TFT-LIGHT-LOGIC: dim by day (TFT_LIGHT), dark TFT_IDLE into a stay, full for TFT_WAKE after "
      "the USER button (even in the dark), dim again when it moves; every change is in the diag, "
      "the first setting is not; TFT_IDLE 0 never goes dark; TFT_LIGHT 0 is dark, the button lights it",
      _stay == [30, 30, 30, 0, 100, 0, 30]
      and _dln == ["tft light 0", "tft light 100", "tft light 0", "tft light 30"]
      and _never == [30, 30] and _zero == [0, 100, 0],
      repr((_stay, _dln, _never, _zero)))
_pressed0 = _ft.pressed
_pr = [True]
_ft.pressed = lambda: _pr[0]
hl._tft, hl._tft_l[:] = _ft, [None, None, None]
hl._diag_write = _dl.append
_ft._D.pwm = None
_pin_levels[:] = []
hl._tft_t[:] = [_T, None, 0, 0, 0]        # the state was made just now: not due for 1 s
hl._tft_l[0] = 30
_cl = _T + 100
hl.tft_tick(_cl)
_tap1 = (hl._tft_l[2], hl._tft_t[0], [p[:2] for p in _pin_levels if p[0] == 21][-1:])
_pr[0] = False
_ft.pressed = _pressed0
hl._tft_l[:] = [None, None, None]
hl._tft, hl._diag_write = _t0, _dw
_ft._D.pwm = None
check("TFT-LIGHT-TAP: a press is seen on the tick it happens in (a tap is short) and lights the "
      "display at once, not up to TFT_STATE later",
      _tap1[0] == _cl and _tap1[2] == [(21, 65535)] and _tap1[1] == _cl, repr(_tap1))
_g1 = (hl.TFT_LIGHT, hl.TFT_IDLE)
hl._tft_env({"TFT_LIGHT": "55", "TFT_IDLE": "120"})
_e1 = (hl.TFT_LIGHT, hl.TFT_IDLE)
hl._tft_env({"TFT_LIGHT": "101", "TFT_IDLE": "x"})       # bad: kept
hl._tft_env({"TFT_LIGHT": "-1", "TFT_IDLE": "86401"})
_e2 = (hl.TFT_LIGHT, hl.TFT_IDLE)
hl._tft_env({"TFT_LIGHT": "0", "TFT_IDLE": "0"})
_e3 = (hl.TFT_LIGHT, hl.TFT_IDLE)
hl._tft_env({})
_e4 = (hl.TFT_LIGHT, hl.TFT_IDLE)
hl.TFT_LIGHT, hl.TFT_IDLE = _g1
check("TFT-LIGHT-ENV: TFT_LIGHT 0-100 and TFT_IDLE 0-86400 s read from .env; out of range or not a "
      "number keeps what was; 0 is allowed for both",
      _e1 == (55, 120000) and _e2 == (55, 120000) and _e3 == (0, 0) and _e4 == (0, 0),
      repr((_e1, _e2, _e3, _e4)))

# --- review 2026-11-08b: GPS/TFT (RG-n) --------------------------------------
_rg_td = tempfile.mkdtemp(prefix="rg-")
_rg_saved = (hl.LOG_DIR, hl.DIAG_FILE, hl.DIAG_OLD, hl._diag_write)
hl.LOG_DIR = os.path.join(_rg_td, "trk")
hl.DIAG_FILE, hl.DIAG_OLD = os.path.join(_rg_td, "diag"), os.path.join(_rg_td, "diag.1")
os.makedirs(hl.LOG_DIR)
_rg_k = [0]


def _rg_reset():
    hl.reset()
    hl.log_reset()
    hl.wifi_reset()
    hl.up_reset()
    hl.wifi.update(state="offline", home=False)
    hl.uart = _UcUart()
    _uart_rx[:] = b""
    hl.st["settled"] = True
    _clock[0] = (_clock[0] // 1000 + 1) * 1000
    _rg_k[0] = 0
    try:
        os.remove(hl.DIAG_FILE)
    except OSError:
        pass


def _rg_sec(t, date="041126", tg=None):
    """One second of a moving fix (about 4 m a second) through the parser and
    log_tick; tg: the time the GGA says (a corrupt one), its position 18 m off."""
    _rg_k[0] += 1
    r = "4711.%04d" % (_rg_k[0] * 20)
    g = "4711.%04d" % (_rg_k[0] * 20 + (100 if tg else 0))
    _uc_feed(hl, _nmea("GNRMC,%s,A,%s,N,01825.6160,E,5.0,,%s,,,A,V" % (t, r, date)),
             _nmea("GNGGA,%s,%s,N,01825.6160,E,1,14,0.80,120.5,M,40.0,M,," % (tg or t, g)))
    hl.log_tick()
    _clock[0] += 1000


def _rg_times():
    return [l[-17:-1] for l in hl.log["batch"]]


def _rg_diag():
    try:
        with open(hl.DIAG_FILE) as f:
            return f.read()
    except OSError:
        return ""


# RG-1: a corrupt digit past the checksum that is a real time, 50 min ahead (the
# minutes of a GGA): its fix was recorded and became last_t, and nothing was
# written until the real time caught up (TIME_BACK was an hour)
_rg_reset()
for _i in range(50):
    _rg_sec("1200%02d.00" % _i, tg="125003.00" if _i == 3 else None)
_rg_tt = _rg_times()
_rg_dg = _rg_diag()
check("RG-1: a fix 50 min ahead (a corrupt digit past the checksum) lets go after "
      "TIME_BACK_N fixes: the track goes on, the false line is gone, a diag line says so",
      hl.TIME_BACK <= 60 and hl.log["last_t"] == "2026-11-04T12:00:48Z"
      and "20261104T125003Z" not in _rg_tt and "20261104T120048Z" in _rg_tt
      and len(_rg_tt) >= 50 - hl.TIME_BACK_N - 3 and "time back" in _rg_dg,
      repr((hl.log["last_t"], len(_rg_tt), _rg_dg[-120:])))
# ... and a fix some seconds behind (jitter, a step of the clock) never counts
_rg_reset()
hl.st.update(date="2026-11-04", time="12:00:00")
hl.log["last_t"] = "2026-11-04T12:00:50Z"
for _i in range(30):
    hl._time_back("2026-11-04T12:00:00Z", hl.log["last_t"])
_rg_b0 = (hl.log["last_t"], hl.log["back"])
hl.log["last_t"] = "2026-11-04T12:01:01Z"
for _i in range(hl.TIME_BACK_N - 1):
    hl._time_back("2026-11-04T12:00:00Z", hl.log["last_t"])
_rg_b1 = hl.log["last_t"]
hl._time_back("2026-11-04T12:00:00Z", hl.log["last_t"])
check("RG-1: 50 s behind never lets go (the clock may step), 61 s behind does after "
      "TIME_BACK_N fixes in a row, not before",
      _rg_b0 == ("2026-11-04T12:00:50Z", 0) and _rg_b1 == "2026-11-04T12:01:01Z"
      and hl.log["last_t"] is None, repr((_rg_b0, _rg_b1, hl.log["last_t"])))

# RG-2: no real time (month 13, day 32, hour 25, minute 61) is no time
_rg_dts = [hl._dt_s(d, t) for d, t in (
    ("2026-13-32", "12:00:00"), ("2026-11-04", "25:00:00"), ("2026-11-04", "12:61:00"),
    ("2026-11-04", "12:00:61"), ("2026-02-29", "00:00:00"), ("2027-02-29", "00:00:00"),
    ("2026-04-31", "00:00:00"), ("2019-12-31", "23:59:59"), ("2100-01-01", "00:00:00"),
    ("2026-11-0x", "12:00:00"), ("", "12:00:00"))]
_rg_ok = (hl._dt_s("2028-02-29", "23:59:59"), hl._dt_s("2026-11-04", "12:00:05"),
          hl._dt_s("2026-11-04T12:00:05Z", "2026-11-04T12:00:05Z", 11))
check("RG-2: _dt_s is None for a date or a time that does not exist (and outside 2020-2099), "
      "as _kb_time says; real ones are as before",
      _rg_dts == [None] * 11 and _rg_ok == (
          calendar.timegm((2028, 2, 29, 23, 59, 59)) - calendar.timegm((2020, 1, 1, 0, 0, 0)),
          calendar.timegm((2026, 11, 4, 12, 0, 5)) - calendar.timegm((2020, 1, 1, 0, 0, 0)),
          calendar.timegm((2026, 11, 4, 12, 0, 5)) - calendar.timegm((2020, 1, 1, 0, 0, 0))),
      repr((_rg_dts, _rg_ok)))
_rg_reset()
for _i in range(20):
    if _i == 2:
        _rg_sec("120002.00", date="321326")          # day 32, digits only
    elif _i == 8:
        _rg_sec("256100.00")                         # hour 25, minute 61
    else:
        _rg_sec("1200%02d.00" % _i)
_rg_tt = _rg_times()
check("RG-2: a second whose date or time does not exist is skipped alone: no line, no clock, "
      "no 10 s gap (the sentences are not errors: the checksum held)",
      len(_rg_tt) == 17 and hl.log["last_t"] == "2026-11-04T12:00:18Z"
      and hl.st["err"] == 0 and hl.log["clock"][0] == hl._dt_s("2026-11-04", "12:00:18")
      and not any(x[4:6] == "13" or x[9:11] == "25" for x in _rg_tt),
      repr((len(_rg_tt), hl.log["last_t"], hl.st["err"], hl.log["clock"])))

# RG-3: the display goes dark though the PWM cannot be set; start() twice
class _RgBadPwm(object):
    def duty_u16(self, d=None):
        raise ValueError("pwm")


_ft._D.bl, _ft._D.pwm = 21, _RgBadPwm()
_pin_levels[:] = []
_rg_oe = None
try:
    _ft.off()
except Exception as e:
    _rg_oe = e
_rg_off = ([p[:2] for p in _pin_levels if p[0] == 21][-1:], _ft._D.pwm, _rg_oe)
_Pwm.live.clear()
try:
    _ft.start(30, 21)
    _rg_p1 = _Pwm.live.get(21)
    _ft.start(30, 21)                     # again: a PWM made anew, at the level
    _rg_p2 = _Pwm.live.get(21)
except Exception:
    _rg_p1 = _rg_p2 = None
_rg_st = (_rg_p1 is not None and _rg_p2 is not _rg_p1 and _rg_p2.duty == 30 * 65535 // 100
          and _ft._D.pwm is _rg_p2)
_ft._D.pwm = None
check("RG-3: off() ends with the pin low when the PWM raises (taken back as an output, "
      "the PWM forgotten); start() again makes the PWM anew at the level it is given",
      _rg_off == ([(21, 0)], None, None) and _rg_st, repr((_rg_off, _rg_st)))
_rg_t0, _rg_p0, _rg_d0 = hl._tft, _ft.pressed, hl._diag_write
hl._tft = _ft
hl._tft_t[:] = [None, None, 0, 0, 0]
hl._tft_l[:] = [None, None, None]
hl._diag_write = lambda l: None


def _rg_boom():
    raise ValueError("pin")


_ft.pressed = _rg_boom
_rg_e = None
try:
    hl.tft_tick(_clock[0])
    hl.tft_tick(_clock[0] + 1)
except Exception as e:
    _rg_e = e
_rg_ok3 = (_rg_e, hl._tft is _ft, hl._tft_t[2:])
_ft.pressed = _rg_p0
hl._tft, hl._diag_write = _rg_t0, _rg_d0
hl._tft_t[:] = [None, None, 0, 0, 0]
check("RG-3: a button that raises leaves tft_tick quietly, the display and its counts are "
      "as they were (nothing leaves tft_tick)",
      _rg_ok3 == (None, True, [0, 0, 0]), repr(_rg_ok3))

# RG-4: the next NTP era only for a young build
class _RgNtpSock(object):
    reply, ev = 0, []
    SOCK_DGRAM = 2

    def __init__(self, *a):
        self.q = None

    @staticmethod
    def getaddrinfo(*a):
        _RgNtpSock.ev.append("dns")
        return [(2, 2, 0, "", ("192.0.2.1", 123))]

    def settimeout(self, x):
        pass

    def sendto(self, q, a):
        self.q = bytes(q)

    def recv(self, n):
        r = bytearray(48)
        r[0], r[1] = 0x24, 2
        r[24:32] = self.q[40:48]
        r[40:44] = _RgNtpSock.reply.to_bytes(4, "big")
        return bytes(r)

    def close(self):
        pass


_RgNtpSock.socket = _RgNtpSock
_rg_u = lambda y, mo, d: calendar.timegm((y, mo, d, 0, 0, 0))
_rg_v = lambda u: (u + 2208988800) % (1 << 32)       # NTP seconds, 32 bits
_rg_s = hl.socket, hl.FW_BUILD, hl.feed
hl.socket = _RgNtpSock
hl.feed = lambda: _RgNtpSock.ev.append("feed")


def _rg_ask(v, build):
    _RgNtpSock.reply, hl.FW_BUILD = v, build
    return hl._ntp_ask()


_rg_ev = None
_rg_a = [_rg_ask(_rg_v(_rg_u(2026, 11, 20)), _rg_u(2026, 11, 8)),
         _rg_ask(10, _rg_u(2026, 11, 8)),                     # 1900: not 2036
         _rg_ask(10 + 3 * 86400, _rg_u(2026, 11, 8)),
         _rg_ask(1000000000, _rg_u(2026, 11, 8)),             # 1931: not 2067
         _rg_ask(_rg_v(_rg_u(2037, 6, 1)), _rg_u(2033, 1, 1)),  # a real 2037
         _rg_ask(_rg_v(_rg_u(2037, 6, 1)), _rg_u(2026, 11, 8)),  # an old build
         _rg_ask(0, _rg_u(2033, 1, 1))]
_RgNtpSock.ev[:] = []
_rg_ask(_rg_v(_rg_u(2026, 11, 20)), _rg_u(2026, 11, 8))
_rg_ev = list(_RgNtpSock.ev)
hl.socket, hl.FW_BUILD, hl.feed = _rg_s
_rg_d20 = lambda u: u - _rg_u(2020, 1, 1)
check("RG-4: _ntp_ask believes the next era only within NTP_ERA of the build (1900 and 1931 "
      "are no 2036 and 2067; a real 2037 is, for a build of 2033); feed() before and "
      "after the name lookup",
      _rg_a == [_rg_d20(_rg_u(2026, 11, 20)), None, None, None, _rg_d20(_rg_u(2037, 6, 1)),
                None, None] and _rg_ev == ["feed", "dns", "feed"], repr((_rg_a, _rg_ev)))

# RG-5: a board on a desk without a usable fix goes dark too; no stamp ages past 2**30 ms
_lt = _LightTft()
_rg_t0, _rg_g, _rg_d0 = hl._tft, (hl.TFT_LIGHT, hl.TFT_IDLE, hl.TFT_WAKE), hl._diag_write
hl._tft, hl._diag_write = _lt, lambda l: None
hl.TFT_LIGHT, hl.TFT_IDLE, hl.TFT_WAKE = 30, 300000, 60000
_rg_reset()
hl._tft_l[:] = [None, None, None]
_rg_sl = hl.log["still"]
hl.wifi.update(state="connected", home=True)
_T = 7000000
_rg_d = []
for _dd in (0, 299000, 301000):
    hl._tft_light(_T + _dd)
    _rg_d.append(_lt.lv[-1])
_rg_sl["last_at"] = _T + 302000                # a usable fix: not a stay
hl._tft_light(_T + 302000)
_rg_d.append(_lt.lv[-1])
_rg_sl["last_at"] = None
hl._tft_light(_T + 303000)                 # no fix again: a stay, TFT_IDLE from now
_rg_d.append(_lt.lv[-1])
hl._tft_light(_T + 303000 + 301000)
_rg_d.append(_lt.lv[-1])
hl.wifi.update(state="offline", home=False)   # away without a fix: lit (in a bag, a tunnel)
hl._tft_light(_T + 700000)
_rg_d.append(_lt.lv[-1])
hl._tft_light(_T + 700000 + 3600000)
_rg_d.append(_lt.lv[-1])
check("RG-5: at home on the home network with no usable fix for STILL_GAP counts as a stay "
      "(dark after TFT_IDLE); a usable fix, or being away, does not",
      _rg_d == [30, 30, 0, 30, 30, 0, 30, 30], repr(_rg_d))
# 13 days of stay and one press of the button, one tick every 30 s, on a wrapping clock
_wrap = 1 << 30
_td0, _ta0 = time.ticks_diff, time.ticks_add
time.ticks_diff = lambda a, b: (a - b + (_wrap >> 1)) % _wrap - (_wrap >> 1)
time.ticks_add = lambda a, b: (a + b) % _wrap
_rg_reset()
hl.wifi.update(state="offline", home=False)
hl.log["still"]["active"] = True
_lt.lv[:] = []
hl._tft_l[:] = [None, None, None]
_n = 0
try:
    hl._tft_l[2] = 5000
    for _n in range(13 * 86400 // 30 + 40):
        hl._tft_light((5000 + _n * 30000) % _wrap)
finally:
    time.ticks_diff, time.ticks_add = _td0, _ta0
_rg_w = list(_lt.lv)
hl.log["still"]["active"] = False
hl._tft, hl._diag_write = _rg_t0, _rg_d0
hl.TFT_LIGHT, hl.TFT_IDLE, hl.TFT_WAKE = _rg_g
hl._tft_l[:] = [None, None, None]
check("RG-5: after 13 days of stay (12.4 d is 2**30 ms) the display is still dark, and the "
      "button of day 0 does not light it again",
      _rg_w == [100, 30, 0], repr(_rg_w))

# RG-7: the history trims whole days; a GSA without a system id
_rg_h = os.path.join(_rg_td, "hist")
os.makedirs(_rg_h)
_rg_hs = hl.HIST_DIR, hl.free
hl.HIST_DIR = _rg_h
for _n in ("20261101.trb", "20261101.trk", "20261102.trb", "20261103.trb"):
    with open(os.path.join(_rg_h, _n), "w") as f:
        f.write("x")
hl.free = lambda: 0 if len(os.listdir(_rg_h)) > 3 else 1 << 30
hl._hist_trim(1000)
_rg_a1 = sorted(os.listdir(_rg_h))
hl.free = lambda: 0
hl._hist_trim(1000)
_rg_a2 = sorted(os.listdir(_rg_h))
shutil.rmtree(_rg_h)
os.makedirs(_rg_h)
for _n in ("20261103.trb", "20261103.trk"):
    with open(os.path.join(_rg_h, _n), "w") as f:
        f.write("x")
hl._hist_trim(1000)
_rg_a3 = sorted(os.listdir(_rg_h))
hl.HIST_DIR, hl.free = _rg_hs
check("RG-7: _hist_trim drops a day at a time (a day's .trb and .trk go together) and keeps "
      "the newest day whole, whatever it holds",
      _rg_a1 == ["20261102.trb", "20261103.trb"] and _rg_a2 == ["20261103.trb"]
      and _rg_a3 == ["20261103.trb", "20261103.trk"], repr((_rg_a1, _rg_a2, _rg_a3)))
_rg_u2 = []
for _tk, _bd in (("GP", "GPGSA,A,3,01,02,03,,,,,,,,,,1.5,0.9,1.2"),
                 ("GL", "GLGSA,A,3,65,66,,,,,,,,,,,1.5,0.9,1.2"),
                 ("GN", "GNGSA,A,3,01,02,,,,,,,,,,,1.5,0.9,1.2"),
                 ("GN", "GNGSA,A,3,07,08,,,,,,,,,,,1.5,0.9,1.2,3")):
    hl._uc_sv["gsa"] = None
    _uc_feed(hl, _nmea(_bd))
    _rg_u2.append(list(hl._uc_sv["used"]))
check("RG-7: a GSA before NMEA 4.10 (no system id) counts for its talker's system, a GN one "
      "for none; with the id (4.10) as before",
      _rg_u2 == [[1, 2, 3], [6 << 8 | 65, 6 << 8 | 66], [], [2 << 8 | 7, 2 << 8 | 8]],
      repr(_rg_u2))
hl.LOG_DIR, hl.DIAG_FILE, hl.DIAG_OLD, hl._diag_write = _rg_saved
shutil.rmtree(_rg_td, ignore_errors=True)

# OTA: each board only its own firmware
_mf = {"build": 5, "commit": "abc1234", "size": 9, "sha256": "0" * 64,
       "url": "x.bin", "files": []}
_mfs = dict(_mf, board="GPS_LOGGER_S3")
check("OTA-BOARD: the board takes only a manifest naming it, and the name is signed",
      hl._ota_manifest_ok(_mfs) and not hl._ota_manifest_ok(_mf)
      and not hl._ota_manifest_ok(dict(_mf, board="GPS_LOGGER_C6"))
      and hl._ota_signed(_mfs) == hl._ota_signed(_mf) + b"|board=GPS_LOGGER_S3")
_uart_rx[:] = b""

# --- 2026-11 review: robustness (RB-n) and guards the suite did not notice
# being removed (GD-n) ---------------------------------------------------------
gl = _reload_logger()
_rb_dir = tempfile.mkdtemp(prefix="rb-")
gl.LOG_DIR = os.path.join(_rb_dir, "gpx")
gl.HIST_DIR = os.path.join(_rb_dir, "hist")
os.makedirs(gl.LOG_DIR)
os.makedirs(gl.HIST_DIR)
gl.reset()
gl.log_reset()
gl.wifi_reset()
gl.up_reset()


class _RbConn(object):
    """An accepted client: what it asked for, what it got."""
    def __init__(self, req=b""):
        self.req, self.sent, self.closed = req, bytearray(), False
    def settimeout(self, t):
        pass
    def recv(self, n):
        out, self.req = self.req[:n], self.req[n:]
        return out
    def write(self, b):
        self.sent.extend(b)
        return len(b)
    def send(self, b):
        self.sent.extend(b)
        return len(b)
    sendall = send
    def close(self):
        self.closed = True


class _RbListen(object):
    """The listening socket: bind/listen as asked, failing at the name in
    oom with a MemoryError, accept what waits, EAGAIN otherwise."""
    oom = None
    made = []
    def __init__(self):
        self.waiting, self.accepts, self.closed = [], 0, False
        _RbListen.made.append(self)
    def _oom(self, name):
        if _RbListen.oom == name:
            raise MemoryError("memory allocation failed")
    def setsockopt(self, *a):
        pass
    def bind(self, a):
        self._oom("bind")
    def listen(self, n):
        self._oom("listen")
    def setblocking(self, b):
        pass
    def accept(self):
        self._oom("accept")
        self.accepts += 1
        if not self.waiting:
            raise OSError(11)
        return self.waiting.pop(0), ("1.2.3.4", 5)
    def close(self):
        self.closed = True


class _RbSockMod(object):
    AF_INET, SOCK_STREAM, SOL_SOCKET, SO_REUSEADDR = 2, 1, 1, 4
    @staticmethod
    def socket(*a):
        if _RbListen.oom == "socket":
            raise MemoryError("memory allocation failed")
        return _RbListen()
    @staticmethod
    def getaddrinfo(host, port):
        return [(2, 1, 0, "", (host, port))]


def _gm_zero():
    """Every guarded MemoryError count to 0 (one list entry before RN-3)."""
    gm = gl._guard_mem
    for k in (gm if isinstance(gm, dict) else range(len(gm))):
        gm[k] = [0, None] if isinstance(gm[k], list) else 0


def _gm_set(name, n):
    """A tick's guarded MemoryError count n, the last one just now."""
    gl._guard_mem[name] = [n, gl.time.ticks_ms()]


def _rb_web(home=False, away=False):
    """The server closed, WEB on, connected; home and WEB_AWAY as asked."""
    gl._web_close()
    gl.socket = _RbSockMod
    gl.WEB, gl.WEB_AWAY = True, away
    gl.wifi_reset()
    gl.wifi.update(state="connected", ip="192.168.1.60", home=home)
    del _RbListen.made[:]
    _RbListen.oom = None
    gl.ota["pend"] = None
    gl._tick_bug.clear()
    _gm_zero()


_rb_socket = gl.socket

# RB-WEB-NET: web_tick serves on whatever network the radio is on. WEB_AWAY
# only keeps the radio on for it off home (README, status page; commit
# 7276921): the page "up only briefly" off home is this, so it stays
_rb_rows = []
for _h, _a in ((True, False), (False, False), (False, True)):
    _rb_web(_h, _a)
    gl.web_tick()
    _c = _RbConn(b"GET /nope HTTP/1.1\r\nHost: 192.168.1.60\r\n\r\n")
    _RbListen.made[-1].waiting.append(_c)
    gl.web_tick()
    _s1 = bytes(_c.sent[:12])
    gl.wifi["state"] = "sleep"           # (asleep, on any network: closed)
    gl.web_tick()
    _rb_rows.append((_s1, _RbListen.made[-1].closed, gl._web_sock is None))
check("RB-WEB-NET: connected, home or not, WEB_AWAY or not, the page answers; "
      "with the radio asleep the socket is closed",
      _rb_rows == [(b"HTTP/1.0 404", True, True)] * 3, repr(_rb_rows))

# RB-OOM: one MemoryError in the page's accept()/socket setup or a status
# line went through run()'s crash branch: a reset, a boot-fail and, an OTA
# pending, /ota.mark "crash" blaming a good build


def _rb_loop(n=5):
    """n passes of _loop; "ran" or what left it (run()'s crash branch)."""
    k = [0]
    def _sl(ms):
        _clock[0] += ms
        k[0] += 1
        if k[0] >= n:
            raise KeyboardInterrupt
    saved = (gl.time.sleep_ms, gl.wifi_tick, gl.upload_tick, gl.ota_tick,
             gl.log_tick, gl.gps_check, gl.keepalive_tick, gl.tft_tick,
             gl._boot_ok, gl.read_uart)
    gl.time.sleep_ms = _sl
    gl.wifi_tick = gl.upload_tick = gl.log_tick = lambda: None
    gl._boot_ok = gl.read_uart = lambda: None
    gl.ota_tick = gl.gps_check = gl.keepalive_tick = gl.tft_tick = lambda now: None
    gl._wdt = None
    try:
        gl._loop()
        return "returned"
    except KeyboardInterrupt:
        return "ran"
    except Exception as e:
        return repr(e)
    finally:
        (gl.time.sleep_ms, gl.wifi_tick, gl.upload_tick, gl.ota_tick,
         gl.log_tick, gl.gps_check, gl.keepalive_tick, gl.tft_tick,
         gl._boot_ok, gl.read_uart) = saved


def _rb_oom(*a):
    raise MemoryError("memory allocation failed")


_rb_marks = []
_rb_saved = (gl._ota_mark, gl.sv_seen, gl._temp, gl.DIAG, gl.STATUS)
gl._ota_mark = _rb_marks.append
_rb_res = {}
try:
    for _p in (None, 200):
        gl.DIAG = 0
        _rb_web()
        gl.web_tick()                    # (the listener, for accept)
        gl.ota["pend"] = _p
        _RbListen.oom = "accept"
        _rb_res["accept", _p] = _rb_loop()
        for _at in ("socket", "bind", "listen"):
            _rb_web()
            gl.ota["pend"] = _p
            _RbListen.oom = _at
            _rb_res[_at, _p] = (_rb_loop(), gl._web_sock is None,
                                all(s.closed for s in _RbListen.made))
        _rb_web()
        gl.WEB = False
        gl.ota["pend"] = _p
        gl.DIAG = 1
        gl.sv_seen = _rb_oom
        _rb_res["diag", _p] = _rb_loop()
        gl.sv_seen = _rb_saved[1]
        gl.DIAG = 0
        gl.STATUS = 1
        gl._tick_bug.clear()
        gl._temp = _rb_oom
        _rb_res["status", _p] = _rb_loop()
        gl._temp = _rb_saved[2]
        gl.STATUS = _rb_saved[4]
finally:
    gl.ota["pend"] = None
    _RbListen.oom = None
    gl._ota_mark, gl.sv_seen, gl._temp, gl.DIAG, gl.STATUS = _rb_saved
_rb_want = {}
for _p in (None, 200):
    _rb_want["accept", _p] = _rb_want["diag", _p] = _rb_want["status", _p] = "ran"
    for _at in ("socket", "bind", "listen"):
        _rb_want[_at, _p] = ("ran", True, True)
check("RB-OOM: a MemoryError in accept(), _web_open (its socket closed), _diag or "
      "_status stays in the loop, also with an OTA pending: no crash branch, no "
      "/ota.mark", _rb_res == _rb_want and not _rb_marks, repr((_rb_res, _rb_marks)))

# RB-OOM-RULES: _guarded's other rules, as they were: a pending image's real
# bug still rolls it back; the page sits out TICK_PAUSE; memory reboots count
_rb_web()
gl.ota["pend"] = 200
_rb_saved = (gl._temp, gl.STATUS)
gl.STATUS = 1
gl._temp = lambda: (_ for _ in ()).throw(TypeError("bug"))
_rb_bug = _rb_loop()
gl._temp, gl.STATUS = _rb_saved
gl.ota["pend"] = None
gl.WEB = True
_rb_web()
gl.web_tick()
_RbListen.oom = "accept"
_rb_acc = []
_rb_real = _RbListen.accept
def _rb_counted(self):
    _rb_acc.append(_clock[0])
    return _rb_real(self)
_RbListen.accept = _rb_counted
_rb_r1 = _rb_loop(int(gl.TICK_PAUSE // 20) + 50)
_RbListen.accept = _rb_real
_RbListen.oom = None
_rb_gap = _rb_acc[1] - _rb_acc[0] if len(_rb_acc) > 1 else None
check("RB-OOM-RULES: an OTA pending, a real bug in a status line still leaves the "
      "loop; a page out of memory waits TICK_PAUSE, no retry every pass",
      "TypeError" in _rb_bug and _rb_r1 == "ran" and len(_rb_acc) == 2
      and _rb_gap is not None and gl.TICK_PAUSE <= _rb_gap <= gl.TICK_PAUSE + 100,
      repr((_rb_bug, _rb_r1, _rb_acc)))

_rb_reboots = []
_rb_saved = (gl._mem_reboot, gl._mem_reboot_soon, gl._bug_once)
gl._mem_reboot = _rb_reboots.append
gl._mem_reboot_soon = lambda: False
_gm = gl._guard_mem
try:
    gl._tick_bug.clear()
    _gm_set("web", gl.GUARD_MEM_MAX - 2)
    gl._guarded("web", lambda: None)
    _rb_kept = _gm["web"][0]        # a page tick running through: no proof
    gl.ota["pend"] = 200
    _gm_set("web", gl.GUARD_PEND_MAX - 1)
    try:                            # (a persistent one with an image pending
        gl._guarded("web", _rb_oom)  # raises: RN2-1; no memory reboot, a
        _rb_pend = "swallowed"      # reset would abort the image)
    except MemoryError:
        _rb_pend = "raised"
    _rb_pend = (_rb_pend, list(_rb_reboots))
    gl.ota["pend"] = None
    gl._tick_bug.clear()
    _gm_set("web", gl.GUARD_MEM_MAX - 1)
    gl._guarded("web", _rb_oom)
    _rb_due = list(_rb_reboots)
    gl._tick_bug.clear()
    _gm_set("wifi", 3)
    gl._guarded("wifi", lambda: None)
    _rb_clear = _gm["wifi"][0]           # (a pass clears no count: RN2-1)
    # the telling allocates too: out of memory it stays quiet, in the loop
    gl._tick_bug.clear()
    gl._bug_once = _rb_oom
    _rb_tell = "ok"
    try:
        gl._guarded("diag", _rb_oom, 1)
    except MemoryError:
        _rb_tell = "raised"
finally:
    gl.ota["pend"] = None
    gl._mem_reboot, gl._mem_reboot_soon, gl._bug_once = _rb_saved
    _gm_zero()
    gl._tick_bug.clear()
check("RB-OOM-COUNT: a tick that runs through clears no count of a lasting "
      "MemoryError, no memory reboot with an image pending (it raises at "
      "GUARD_PEND_MAX), one at GUARD_MEM_MAX without; a MemoryError in the "
      "telling stays in",
      _rb_kept == gl.GUARD_MEM_MAX - 2 and _rb_pend == ("raised", [])
      and _rb_due == ["guarded memory web"] and _rb_clear == 3
      and _rb_tell == "ok",
      repr((_rb_kept, _rb_pend, _rb_due, _rb_clear, _rb_tell)))

# RB-KB-FEED: _kb_seek from the start of a long .trb, on no feed
_rb_p = os.path.join(gl.LOG_DIR, "feed.trb")
_rb_lines = _tb_drive(2000, 2)
_rb_body = gl.HEADERB.encode() + _kb_encode(_rb_lines, len(_rb_lines), None)
with open(_rb_p, "wb") as f:
    f.write(_rb_body)
_rb_s = list(gl._ks)
gl._wdt = WDT(timeout=120000)
gl._wdt.fed = 0
with builtins.open(_rb_p, "rb") as f:    # (this file's open decodes a .trb)
    _rb_k = gl._kb_seek(f, len(_rb_body), -1, _rb_s)
_rb_fed = gl._wdt.fed
gl._wdt = None
check("RB-KB-FEED: _kb_seek feeds the WDT every 64 records (a day's file from its "
      "start), and still finds the end",
      _rb_k is not None and _rb_fed >= 2000 // 64 - 1, repr((_rb_k, _rb_fed)))

# RB-UART: read_uart has a budget: a UART that never empties must not hold
# the loop, its feed() and the logging. In normal use nothing is lost


class _RbUart(object):
    """data as it would wait in the UART's buffer; flood: never empty."""
    def __init__(self, data=b"", flood=False):
        self.data, self.flood, self.reads = bytearray(data), flood, 0
    def any(self):
        return 1 if self.flood else len(self.data)
    def readinto(self, mv):
        self.reads += 1
        if self.flood:
            if self.reads > 20000:       # (the old endless loop: ends the test)
                self.flood = False
            mv[:6] = b"$junk\n"
            return 6
        k = min(len(mv), len(self.data))
        mv[:k] = self.data[:k]
        del self.data[:k]
        return k


_rb_saved = (gl.uart, gl.handle)
_rb_seen = []
gl.handle = lambda b, a, e: _rb_seen.append(bytes(b[a:e]))
try:
    gl.reset()
    gl._rx_n = gl._rx_s = 0
    gl.uart = _RbUart(flood=True)
    gl.read_uart()
    _rb_flood = gl.uart.reads
    # a full 8 KB buffer (the Heltec's): one pass takes all of it, in order
    gl._rx_n = gl._rx_s = 0
    del _rb_seen[:]
    _rb_ln = [b"$GNGGA,%05d,47.1,N,18.4,E,1,12,0.9,100,M,40,M,,*00" % i for i in range(160)]
    _rb_8k = b"\n".join(_rb_ln) + b"\n"
    gl.uart = _RbUart(_rb_8k)
    gl.read_uart()
    _rb_one = (len(_rb_8k) >= 8192, gl.uart.any(), _rb_seen == _rb_ln)
    # more than a pass takes: the rest waits there, no byte lost or doubled
    gl._rx_n = gl._rx_s = 0
    del _rb_seen[:]
    _rb_ln = [b"$GNGGA,%05d,47.1,N,18.4,E,1,12,0.9,100,M,40,M,,*00" % i for i in range(1500)]
    gl.uart = _RbUart(b"\n".join(_rb_ln) + b"\n")
    _rb_n = 0
    while gl.uart.any() and _rb_n < 100:
        gl.read_uart()
        _rb_n += 1
    _rb_many = (_rb_n > 1, _rb_seen == _rb_ln)
finally:
    gl.uart, gl.handle = _rb_saved
    gl._rx_n = gl._rx_s = 0
    gl.reset()
check("RB-UART: a UART never empty: read_uart returns after its budget",
      _rb_flood <= getattr(gl, "RX_READS", 0) and _rb_flood > 0, repr(_rb_flood))
check("RB-UART-NOLOSS: a full 8 KB buffer goes in one pass (a pass is ~25 ms at "
      "2.3 KB/s); more waits in the UART for the next pass, nothing lost or doubled",
      _rb_one == (True, 0, True) and _rb_many == (True, True),
      repr((_rb_one, _rb_many)))

# RB-POLL: no client: a poll that allocates nothing, no accept() raising
# EAGAIN (~96 B, ~40 times a second). tests/web_loopback.py and
# tests/mpy_unix_logger.py run the real one on the unix port


class _RbPoll(object):
    def __init__(self):
        self.reg, self.timeouts = [], []
    def register(self, s, ev):
        self.reg.append((s, ev))
    def ipoll(self, t=-1):
        self.timeouts.append(t)
        return iter([(s, 1) for s, _e in self.reg if s.waiting])


class _RbSelect(object):
    POLLIN = 1
    @staticmethod
    def poll():
        return _RbPoll()


_rb_saved = gl.select
gl.select = _RbSelect
try:
    _rb_web()
    gl.web_tick()
    _rb_l = _RbListen.made[-1]
    for _i in range(50):
        gl.web_tick()
    _rb_idle = _rb_l.accepts
    _rb_c = _RbConn(b"GET /nope HTTP/1.1\r\n\r\n")
    _rb_l.waiting.append(_rb_c)
    gl.web_tick()
    _rb_pl = getattr(gl, "_web_poll", None)
    _rb_tmo = set(_rb_pl.timeouts) if _rb_pl is not None else None
    gl.wifi["state"] = "sleep"
    gl.web_tick()
    _rb_gone = getattr(gl, "_web_poll", 1)
finally:
    gl.select = _rb_saved
check("RB-POLL: no accept() with nobody waiting, a zero-timeout poll; a client is "
      "served; the poll goes with the socket",
      _rb_idle == 0 and _rb_c.sent[:12] == b"HTTP/1.0 404" and _rb_tmo == {0}
      and _rb_gone is None, repr((_rb_idle, bytes(_rb_c.sent[:12]), _rb_tmo, _rb_gone)))
# ... and where poll has no ipoll (CPython): accept() as before
_rb_web()
gl.web_tick()
_rb_l = _RbListen.made[-1]
_rb_c = _RbConn(b"GET /nope HTTP/1.1\r\n\r\n")
_rb_l.waiting.append(_rb_c)
gl.web_tick()
check("RB-POLL-NONE: no usable poll: accept() asks, as before",
      getattr(gl, "_web_poll", None) is None and _rb_c.sent[:12] == b"HTTP/1.0 404",
      repr(bytes(_rb_c.sent[:12])))
gl._web_close()
gl.socket, gl.WEB, gl.WEB_AWAY = _rb_socket, False, False
gl.wifi_reset()

# RB-Q: no new list on each pass without a fix when the queue is empty
gl.reset()
gl.log_reset()
_rb_q = gl.st["q"]
gl.log_tick()
_rb_same = gl.st["q"] is _rb_q
gl.st["q"] = [("stale",)]
gl.log_tick()
check("RB-Q: without a fix an empty seconds queue keeps its list (a new one every "
      "pass was garbage), a stale second still goes",
      _rb_same and gl.st["q"] == [], repr(gl.st["q"]))

# RB-CELL: a comprehension over skip made it a cell, allocated at every
# upload_tick (32 B, ~40 times a second: tests/mpy_unix_logger.py measures)
import ast as _ast
_rb_cells = [n for n in _ast.walk(_ast.parse(inspect.getsource(gl.upload_tick)))
             if isinstance(n, _ast.ListComp)
             and any(isinstance(x, _ast.Name) and x.id == "skip" for x in _ast.walk(n))]
check("RB-CELL: upload_tick's skip is no cell", not _rb_cells, repr(len(_rb_cells)))

# RB-TFT: _text blits nothing outside the band, the panel stays as it was
_ft_saved = _ft._text


def _ft_all(fb, font, s, x, base, tone, bg=_ft.PAPER):
    """_text as it was: every glyph, into every band (blit clips)."""
    f, mv = font
    p = _ft._gs4_palette(tone, bg)
    y = base - f[5]
    h = f[4]
    for ch in s:
        i = _ft._index(f, ch)
        w = f[1][i]
        o = f[2][i * 2] | f[2][i * 2 + 1] << 8
        fb.blit((mv[o:o + (w + 1) // 2 * h], w, h, _FakeFramebuf.GS4_HMSB), x, y, _ft._COL[bg], p)
        x += w
    return x


_rb_states = [(_fv, "8,6 km", True, 22, "11:42", 3, 3, "1284", True, (42, "7 nap")),
              (None, "", False, 0, "--:--", None, 0, "", False, None),
              (_fv, "99,9 km", True, 9, "23:59", 0, 1, "kész", False, (100, "tele")),
              ((_fv[0], 0, 0, 2), "0,0 km", True, 12, "00:00", 1, 2, "99999", True, (75, "<1 óra"))]


def _rb_panel(text):
    _ft._text = text
    out = []
    try:
        for s in _rb_states:
            _ft._D.top = _ft._D.low = None
            _ft.set_state(*s)
            _ft.frame()
            out.append(dict(_TftSpi.panel))
    finally:
        _ft._text = _ft_saved
    return out


_TftSpi.fail = False
_ft.start(100, 21)
_rb_new = _rb_panel(_ft_saved)
_ft.start(100, 21)
_rb_old = _rb_panel(_ft_all)
_rb_diff = [i for i in range(len(_rb_old)) if _rb_old[i] != _rb_new[i]]


class _RbFb(object):
    """The band, counting the glyphs blitted wholly outside it."""
    def __init__(self, fb):
        self.fb, self.out, self.n = fb, 0, 0
    def blit(self, src, x, y, key=-1, pal=None):
        h = src[2] if isinstance(src, tuple) else 0
        if isinstance(src, tuple) and src[3] == _FakeFramebuf.GS4_HMSB:
            self.n += 1
            if y >= _ft.BAND_H or y + h <= 0:
                self.out += 1
        self.fb.blit(src, x, y, key, pal)
    def __getattr__(self, k):
        return getattr(self.fb, k)


# ... and at the band's very edges, one row in and just out: each font's every
# glyph, and one inked to its first and last row (the fonts' may be blank there)
_rb_edge = []
_rb_ink = ("X", b"\x04", b"\x00\x00", b"\xff" * 12, 6, 5)
for _fnt in _ft._FONTS + [(_rb_ink, memoryview(_rb_ink[3]))]:
    _f = _fnt[0]
    _hh = _f[4]
    for _y in (-_hh, 1 - _hh, _ft.BAND_H - 1, _ft.BAND_H):
        _ft._D.fb.fill(_PAPER_C)
        _ft_saved(_ft._D.fb, _fnt, _f[0], 0, _y + _f[5], _ft.INK)
        _a1 = bytes(_ft._D.buf)
        _ft._D.fb.fill(_PAPER_C)
        _ft_all(_ft._D.fb, _fnt, _f[0], 0, _y + _f[5], _ft.INK)
        if _a1 != bytes(_ft._D.buf):
            _rb_edge.append((_hh, _y))
_rb_fb0 = _ft._D.fb
_ft._D.fb = _RbFb(_rb_fb0)
_ft._D.top = _ft._D.low = None
_ft.set_state(*_rb_states[0])
_ft.frame()
_rb_blit = (_ft._D.fb.n, _ft._D.fb.out)
_rb_x = (_ft._text(_ft._D.fb, _ft._SATS, "12", 7, 500, _ft.INK), 7 + _ft.width(_ft._SATS, "12"))
_ft._D.fb = _rb_fb0
check("RB-TFT: no glyph blitted outside its band; x still follows; the panel "
      "pixel-identical to blitting every glyph into every band",
      not _rb_diff and not _rb_edge and _rb_blit[0] > 0 and _rb_blit[1] == 0
      and _rb_x[0] == _rb_x[1] and len(_rb_new[0]) == 160 * 80,
      repr((_rb_diff, _rb_edge, _rb_blit, _rb_x)))

# RB-HITS: an OTA download the guard aborted raises the same counter: the
# next failed POST said "upload aborted" though nothing of it was


def _rb_guard_hit():
    """As _raw_get's guard fires: the deadline over, its socket closed."""
    gl._guard_start(_RbConn(), 100)
    _clock[0] += 200
    gl._guard_cb(None)
    gl._guard_stop()


gl.DIAG = 60000
diag_clear()
gl.up["url"], gl.up["key"] = "https://x", "k"
_rb_hits0 = gl._guard["hits"]
_rb_guard_hit()
_post_raise[0] = OSError(-202)
gl._http_post(10)
_rb_l1 = [l for l in diag_lines() if l.startswith("upload")]
diag_clear()
# a POST the guard did abort still says so
_rb_saved = gl.HTTP_POST
def _rb_aborted(url, data, headers, timeout):
    gl._guard["hits"] += 1
    raise OSError(9)
gl.HTTP_POST = _rb_aborted
_post_raise[0] = None
gl._http_post(10)
gl.HTTP_POST = _rb_saved
_rb_l2 = [l for l in diag_lines() if l.startswith("upload aborted")]
check("RB-HITS: an aborted OTA download is no 'upload aborted' for the next failed "
      "POST; a POST the guard aborted is",
      gl._guard["hits"] > _rb_hits0 + 1 and _rb_l1 == ["upload err OSError(-202)"]
      and len(_rb_l2) == 1, repr((_rb_l1, _rb_l2)))
diag_clear()

# GD-GC: the early, frequent collection (a heap grown into the IDF heap
# starves TLS: see gps_logger.py). The unix port checks the real gc too
_rb_th = []
_rb_saved = sys.modules["gc"].threshold
sys.modules["gc"].threshold = lambda *a: _rb_th.append(a)
_heltec_logger()
sys.modules["gc"].threshold = _rb_saved
check("GD-GC: importing the logger sets gc.threshold(8192)", _rb_th == [(8192,)],
      repr(_rb_th))

# GD-HIST: /hist's day is a date, not a path (d=../gpx/<name> served any file)
with open(os.path.join(gl.LOG_DIR, "x.trk"), "w") as f:
    f.write("SECRET-TRACK\n")
with open(os.path.join(gl.LOG_DIR, "somelongername.trk"), "w") as f:
    f.write("SECRET-TRACK\n")
with open(os.path.join(gl.HIST_DIR, "20261001.trk"), "w") as f:
    f.write("TODAY-LINE\n")
_rb_hist = []
for _d in (b"../gpx/x", b"../gpx/somelongername", b"20261001"):
    _c = _RbConn()
    gl._web_end[0] = None
    gl._web_reply(_c, b"GET /hist?d=" + _d + b"&o=0 HTTP/1.1\r\n\r\n")
    _rb_hist.append((bytes(_c.sent[:12]), b"SECRET" in _c.sent, b"TODAY" in _c.sent))
check("GD-HIST: a /hist day that is no 8-digit date is a 404 (d=../gpx/<name>), a "
      "real day is served",
      _rb_hist == [(b"HTTP/1.0 404", False, False)] * 2 + [(b"HTTP/1.0 200", False, True)],
      repr(_rb_hist))

# GD-CA: only a missing CA_FILE means "no CA": any other error (a failing
# flash) must not turn into an unverified POST
_rb_saved = (gl.CA_FILE, os.stat)
gl.CA_FILE = os.path.join(_rb_dir, "no-such-ca.der")
_rb_none = gl._ca_size()
_rb_real_stat = os.stat
def _rb_stat(p, *a, **k):
    if p == gl.CA_FILE:
        raise OSError(5, "EIO")
    return _rb_real_stat(p, *a, **k)
os.stat = _rb_stat
gl._tls_ctx[0] = None
_rb_ca = []
try:
    for _fn in (gl._ca_size, gl._tls_context):
        try:
            _fn()
            _rb_ca.append("no error")
        except OSError as e:
            _rb_ca.append(e.args[0])
finally:
    os.stat = _rb_real_stat
    gl.CA_FILE = _rb_saved[0]
    gl._tls_ctx[0] = None
check("GD-CA: _ca_size: ENOENT is None, any other error raises, and the TLS context "
      "is not made without the CA (no unverified POST)",
      _rb_none is None and _rb_ca == [5, 5], repr((_rb_none, _rb_ca)))

# GD-IDLE: the radio does not sleep off home while a new firmware still has to
# read the manifest: a sleep is no retry, a rollback at OTA_HEALTH
_rb_saved = (gl._radio_idle_ok, set(WLAN.ok))
_rb_idle = []
for _p in (200, None):
    gl.WEB = False
    gl.wifi_reset()
    _w = gl.network.WLAN(gl.network.STA_IF)
    _w.active(True)
    WLAN.ok = {"phone"}
    _w.connect("phone", "x")
    gl.wifi.update(wlan=_w, state="connected", home=False, ssid="phone", ip="1.2.3.4")
    gl.wifi["idle_at"] = _clock[0]
    gl.ota["pend"] = _p
    gl._radio_idle_ok = lambda: True
    _clock[0] += gl.WIFI_IDLE
    gl.wifi_tick()
    _rb_idle.append(gl.wifi["state"])
gl.ota["pend"] = None
gl._radio_idle_ok, WLAN.ok = _rb_saved
gl.wifi_reset()
check("GD-IDLE: connected off home with all uploaded: the radio sleeps, but not "
      "while a firmware waits to confirm", _rb_idle == ["connected", "sleep"],
      repr(_rb_idle))

# GD-DIAG-RES: the log never takes the flash reserve kept for the tracks
_rb_saved = (gl.free, gl.DIAG)
gl.DIAG = 60000
diag_clear()
gl.free = lambda: gl.MIN_FREE - 1
gl._diag_write("under the reserve")
_rb_d1 = os.path.exists(gl.DIAG_FILE)
gl.free = lambda: gl.MIN_FREE
gl._diag_write("at the reserve")
_rb_d2 = diag_lines()
gl.free = lambda: None
gl._diag_write("no statvfs")
_rb_d3 = diag_lines()
gl.free, gl.DIAG = _rb_saved
diag_clear()
check("GD-DIAG-RES: under MIN_FREE free no diag line is written (the tracks keep the "
      "reserve), at it or with free unknown one is",
      _rb_d1 is False and _rb_d2 == ["at the reserve"]
      and _rb_d3 == ["at the reserve", "no statvfs"], repr((_rb_d1, _rb_d2, _rb_d3)))

# GD-HOLD: a held stay keeps at most HOLD_MAX lines: a long drive must not
# reach MAX_BATCH (which drops the oldest)


def _rb_batch(n):
    return ["47.500000,19.000000,100.0,2,0.0,20261031T%02d%02d%02dZ\n"
            % (i // 3600, i // 60 % 60, i % 60) for i in range(n)]


gl.reset()
gl.log_reset()
gl.free = lambda: 10 ** 9
gl.log["file"] = os.path.join(gl.LOG_DIR, "hold.trk")
gl.log["created"] = False
_rb_b = _rb_batch(gl.HOLD_MAX + 5)
gl.log["batch"][:] = _rb_b
gl.flush("20261031T000000Z")         # all newer than the hold
_rb_h1 = list(gl.log["batch"])
gl.log["batch"][:] = _rb_batch(gl.HOLD_MAX)
_rb_sz = gl._file_size(gl.log["file"])
gl.flush("20261031T000000Z")
_rb_h2 = (len(gl.log["batch"]), gl._file_size(gl.log["file"]) == _rb_sz)
check("GD-HOLD: flush(hold) lets the oldest lines over HOLD_MAX go to the file, up "
      "to HOLD_MAX it holds them all",
      _rb_h1 == _rb_b[5:] and _rb_h2 == (gl.HOLD_MAX, True), repr((len(_rb_h1), _rb_h2)))

# GD-TRIM: at the reserve an older history day goes before any track line:
# the drawing yields to the points
_rb_days = [os.path.join(gl.HIST_DIR, d + ".trk") for d in ("20261002", "20261003")]
for _f in _rb_days:
    with open(_f, "w") as f:
        f.write("x\n")
gl.free = lambda: 10 ** 9 if not os.path.exists(os.path.join(gl.HIST_DIR, "20261001.trk")) else gl.MIN_FREE - 1
gl.reset()
gl.log_reset()
gl.log["file"] = os.path.join(gl.LOG_DIR, "trim.trk")
gl.log["created"] = False
gl.log["fix"] = True
_rb_b = _rb_batch(3)
gl.log["batch"][:] = _rb_b
gl.flush()
_rb_tr = (gl.log["full"], gl.log["batch"], os.path.exists(gl.log["file"]),
          os.path.exists(os.path.join(gl.HIST_DIR, "20261001.trk")), os.path.exists(_rb_days[-1]))
check("GD-TRIM: flush at the flash reserve removes older history days and writes the "
      "lines; the newest day stays",
      _rb_tr == (False, [], True, False, True), repr(_rb_tr))
gl.free = lambda: 10 ** 9

# GD-POS-NL: a .trk offset not after a newline is inside a line (a damaged
# .pos): from 0, not half a line up
_rb_p = os.path.join(gl.LOG_DIR, "pos.trk")
_rb_body = b"#head\n" + b"".join(l.encode() for l in _rb_batch(3))
with open(_rb_p, "wb") as f:
    f.write(_rb_body)
_rb_nl = _rb_body.index(b"\n") + 1
gl.write_offset(_rb_p, _rb_nl)
_rb_pos1 = gl.read_pos(_rb_p)
gl.write_offset(_rb_p, _rb_nl + 5)
_rb_pos2 = gl.read_pos(_rb_p)
check("GD-POS-NL: a .trk .pos after a newline is kept, one inside a line reads as 0",
      _rb_pos1 == (_rb_nl, None) and _rb_pos2 == (0, None), repr((_rb_pos1, _rb_pos2)))

# GD-WDT-COUNT: main.py counts a WDT reset before it decides to start: a
# logger hanging at boot must end in the REPL, not in a reset loop for ever
_rb_main = {"Pin": Pin, "RTC": RTC, "machine": sys.modules["machine"], "time": time,
            "__name__": "boot_helpers"}
with open(os.path.join(ROOT, "main.py")) as f:
    exec(compile(f.read(), "main.py", "exec"), _rb_main)
_rb_main["gps_boot"] = None
_rb_ran = []
_rb_fake = types.ModuleType("gps_logger")
_rb_fake.run = lambda: _rb_ran.append(1)
_rb_real_gl = sys.modules.get("gps_logger")
sys.modules["gps_logger"] = _rb_fake
_rb_rc = _reset_cause[0], RTC.mem
try:
    _reset_cause[0] = 3                  # WDT_RESET
    RTC.mem = b"%d" % (_rb_main["BOOT_FAIL_MAX"] - 1)
    _rb_main["_boot_start"]()
    _rb_wdt = (list(_rb_ran), _rb_main["_boot_fail_get"]())
    del _rb_ran[:]
    _reset_cause[0] = 1
    RTC.mem = b"%d" % (_rb_main["BOOT_FAIL_MAX"] - 1)
    _rb_main["_boot_start"]()
    _rb_pwr = (list(_rb_ran), _rb_main["_boot_fail_get"]())
finally:
    _reset_cause[0], RTC.mem = _rb_rc
    if _rb_real_gl is not None:
        sys.modules["gps_logger"] = _rb_real_gl
check("GD-WDT-COUNT: _boot_start counts a WDT reset first: the last one allowed does "
      "not start the logger again; any other reset does",
      _rb_wdt == ([], 5) and _rb_pwr == ([1], 4), repr((_rb_wdt, _rb_pwr)))
shutil.rmtree(_rb_dir, ignore_errors=True)

# --- 2026-11-08 review: network, OTA and guard fixes (RN-n) -----------------
_rn_dir = tempfile.mkdtemp(prefix="rn-")
_rn_saved = (gl.esp32, gl.HTTP_GET, gl.OTA_URL, gl.OTA_KEY, gl.FW_BUILD,
             gl.FW_COMMIT, gl.OTA_PENDING, gl.OTA_BAD, gl.machine.reset,
             gl.DIAG, gl.OTA_TRIES, gl.OTA_PAGES, gl.OTA_MARK, gl.OTA_SHADOW,
             gl._mem_reboot, gl._mem_reboot_soon, _ota_dir)
_ota_dir = _rn_dir
gl.esp32, gl.HTTP_GET = _FakeEsp32, _ota_get
gl.OTA_URL, gl.OTA_KEY = "http://ota.lan/gps/manifest.json", b"secret"
gl.FW_BUILD, gl.FW_COMMIT = 100, "abc0100"
for _k in ("PENDING", "BAD", "TRIES", "MARK"):
    setattr(gl, "OTA_" + _k, os.path.join(_rn_dir, "ota." + _k.lower()))
gl.OTA_PAGES = (os.path.join(_rn_dir, "web.html.gz"),)
gl.OTA_SHADOW = (os.path.join(_rn_dir, "gps_logger.mpy"),)
gl.machine.reset = lambda: _ota_log.append(("reset",))
gl.DIAG = 60000
_rn_reb = []
gl._mem_reboot = lambda why: _rn_reb.append(why)
gl._mem_reboot_soon = lambda: False
try:
    gl.wifi_reset()
    gl.log_reset()
    gl.up_reset()
    _gm_zero()
    # RN-1: a MemoryError in a guarded tick while an image is pending is no
    # crash (it went to run()'s crash branch: /ota.mark, the build bad for
    # good, for a low heap in its first minutes); a real bug still is
    gl.ota["pend"] = 200
    rn1 = []
    for exc in (MemoryError(), ValueError("bug")):
        gl._tick_bug.clear()
        try:
            gl._guarded("upload", lambda e=exc: (_ for _ in ()).throw(e))
            rn1.append(("swallowed", "upload" in gl._tick_bug))
        except Exception as e:
            rn1.append(type(e).__name__)
    rn1 += [_gm["upload"][0], list(_rn_reb)]
    gl.ota["pend"] = None
    check("RN-1: a MemoryError in any guarded tick while an OTA image is "
          "pending is gc + TICK_PAUSE, not a crash that blames the build; a "
          "bug still raises",
          rn1 == [("swallowed", True), "ValueError", 1, []], repr(rn1))
    # RN-3: GUARD_MEM_MAX MemoryErrors of one tick in a row reboot although
    # the other ticks run through meanwhile (every pass of the loop: the
    # one count was cleared by any of them, the reboot never came)
    gl._tick_bug.clear()
    _gm_zero()
    del _rn_reb[:]
    for _ in range(gl.GUARD_MEM_MAX):
        gl._tick_bug.clear()
        gl._guarded("upload", lambda: (_ for _ in ()).throw(MemoryError()))
        gl._guarded("wifi", lambda: None)
    rn3 = (list(_rn_reb), _gm["upload"][0], _gm["wifi"][0])
    check("RN-3: a tick's MemoryErrors are counted per tick: GUARD_MEM_MAX of "
          "the upload's in a row reboot while the WiFi tick runs through",
          rn3 == (["guarded memory upload"], 0, 0), repr(rn3))
    gl._tick_bug.clear()
    # RN-2: the image download cut short (a network error, a short body)
    # waits OTA_RETRY like a bad image; it was downloaded again (2 MB, the
    # loop blocked) every OTA_CHECK, with no "failed" note
    _ota_reset()
    _serve(_manifest(200))
    _rn_img = "http://ota.lan/gps/micropython.bin"
    _rn_n = [0]

    def _rn_get(url, sink, limit):
        if url == _rn_img:
            _rn_n[0] += 1
            raise OSError(110)
        return _ota_get(url, sink, limit)
    gl.HTTP_GET = _rn_get
    gl.wifi.update(state="connected", home=True)
    gl.up.update(state="idle", left=0, files=0)
    gl.ota["boot_at"] = _clock[0] - gl.OTA_FIRST
    gl.ota_tick(_clock[0])
    rn2 = [gl.ota["state"], gl.ota["failed"], _rn_n[0]]
    _clock[0] += gl.OTA_CHECK + 1
    gl.ota_tick(_clock[0])
    rn2 += [gl.ota["state"], _rn_n[0]]
    _clock[0] += gl.OTA_RETRY
    gl.ota_tick(_clock[0])
    rn2.append(_rn_n[0])
    gl.HTTP_GET = _ota_get
    check("RN-2: an image download that fails notes the build as failed: "
          "the next check reads the manifest only, the download comes again "
          "after OTA_RETRY",
          rn2[0] == "err" and rn2[1] is not None and rn2[1][0] == 200
          and rn2[1][2] == "download err" and rn2[2:] == [1, "download err",
                                                           1, 2], repr(rn2))
    # RN-4: with no GEO_URL / API_KEY ("no .env") the points on the flash
    # never go up and "left" never falls: the update check was held back
    # for OTA_STUCK (24 h) and then came as "uploads stuck"
    _ota_reset()
    _rn_calls = []
    _rn_sc = gl._ota_check
    gl._ota_check = lambda: _rn_calls.append(1)
    try:
        gl.up.update(state="no .env", left=5, files=0)
        gl.ota["boot_at"] = _clock[0] - gl.OTA_FIRST
        gl.ota_tick(_clock[0])
        rn4 = [len(_rn_calls), gl.ota.get("stuck")]
        gl.up.update(state="idle")       # with a GEO_URL the points hold it
        gl.ota["at"] = _clock[0] - gl.OTA_CHECK
        gl.ota_tick(_clock[0])
        rn4.append(len(_rn_calls))
    finally:
        gl._ota_check = _rn_sc
    check("RN-4: points waiting with no GEO_URL / API_KEY do not hold the "
          "update check back (they would for good); with one they do",
          rn4 == [1, False, 1], repr(rn4))
    # RN-9: an /ota.pending that is there but cannot be read or parsed is
    # reported as (None, "?", []) by _ota_pending; what ota_boot does with
    # it is RN2-7 (dropped, the running image confirmed once)
    _ota_reset()
    gl.FW_BUILD = 200
    with open(gl.OTA_PENDING, "wb") as f:
        f.write(b"\xff\xfe garbage\n")
    rn9 = [gl._ota_pending()]
    os.remove(gl.OTA_PENDING)
    rn9.append(gl._ota_pending())
    gl.FW_BUILD = 100
    check("RN-9: a damaged /ota.pending reads as an unknown build, a missing "
          "one as none", rn9 == [(None, "?", []), None], repr(rn9))
    # RN-5: a head over HTTP_LINES pieces is an error, not the rest of the
    # head read as the body (a GET's Content-Length missed: the manifest no
    # JSON, the image's sha256 wrong at every check, no clue); a POST keeps
    # its status (the head is read for Retry-After and Content-Type only)
    _rn_hdrs = b"".join(b"X-Pad-%d: y\r\n" % i for i in range(gl.HTTP_LINES + 6))
    _rn_saved2 = (gl.socket, gl.read_uart)
    _rn_out = bytearray()
    gl.socket, gl.read_uart = _gsock, lambda: None
    try:
        _gs[0] = _GetSock(b"HTTP/1.0 200 OK\r\n" + _rn_hdrs
                          + b"Content-Length: 4\r\n\r\nBODY")
        try:
            rn5 = gl._raw_get("http://h/x", _rn_out.extend, 100000)
        except OSError as e:
            rn5 = ("OSError", str(e))
    finally:
        gl.socket, gl.read_uart = _rn_saved2

    class _RnSock(RvSock):
        def settimeout(self, t):
            self.tos = getattr(self, "tos", []) + [t]
    rv_sock[0] = _RnSock(b"HTTP/1.0 201 Created\r\n" + _rn_hdrs + b"\r\n",
                         polls=0)
    gl.socket, gl.select = fake_sockmod, fake_sel
    sys.modules["tls"] = RvTls
    tls_ctx_reset()
    try:
        rn5b = gl._raw_post("https://d.example/x", b"{}", {}, 15)
        # RN-8: after POLLIN the status line waits what is left of
        # HTTP_WAIT on the raw socket (a TLS 1.3 server's session tickets
        # are the first POLLIN; the answer then had the 15 s connect
        # timeout only), not the connect timeout
        rv_sock[0] = _RnSock(b"HTTP/1.0 204 No Content\r\n\r\n", polls=3)
        rn8a = gl._raw_post("https://d.example/x", b"{}", {}, 15)
        rn8 = rv_sock[0].tos
    finally:
        gl.socket, gl.select = real_mods[0], real_mods[1]
        sys.modules["tls"] = real_mods[2]
        tls_ctx_reset()
    check("RN-5: an answer head over HTTP_LINES pieces raises in a GET (no "
          "body from the head), a POST keeps its status",
          rn5 == ("OSError", "head over %d lines" % gl.HTTP_LINES)
          and not _rn_out and rn5b == (201, None, None), repr((rn5, rn5b)))
    check("RN-8: the status line's timeout is what is left of HTTP_WAIT "
          "after the poll, set on the raw socket",
          rn8a == (204, None, None) and len(rn8) == 2 and rn8[0] == 15
          and gl.HTTP_WAIT // 1000 - 1 <= rn8[1] <= gl.HTTP_WAIT // 1000,
          repr((rn8a, rn8)))
    # RN-6: with /ca.der and no fix since the boot (indoors, at home) every
    # verified POST failed the certificate's dates until the first fix: the
    # NTP answer sets the clock meanwhile, a fix sets it again when it
    # differs (the fix wins), a stale answer (the ticks wrapped) does not
    _rn_st = dict((k, gl.st.get(k)) for k in ("valid", "date", "time", "lat",
                                              "pos_at"))
    _rn_ntp = list(gl._ntp)
    try:
        gl.st.update(valid=False)
        RTC.dt, RTC.sets[:] = (2000, 1, 1, 5, 0, 0, 0, 0), []
        gl._rtc_set[0] = False
        wd = _dt.date(2026, 10, 31).weekday()
        secs = ((_dt.date(2026, 10, 31) - _dt.date(2020, 1, 1)).days * 86400
                + 12 * 3600)
        diag_clear()
        gl._ntp[0] = (secs, _clock[0] - 5000)
        gl._rtc_sync(_clock[0])
        gl._rtc_sync(_clock[0])          # set already: nothing
        rn6 = [RTC.sets[:], [l for l in diag_lines() if l.startswith("clock")]]
        gl.st.update(valid=True, date="2026-10-31", time="12:00:25", lat=47.1,
                     pos_at=_clock[0])
        gl._rtc_sync(_clock[0])          # a fix 20 s off: the fix wins
        rn6.append(RTC.sets[1:])
        gl.st.update(valid=False)
        RTC.sets[:] = []
        gl._rtc_set[0] = False
        gl._ntp[0] = (secs, _clock[0] - gl.TICKS_OLD - 10)
        gl._rtc_sync(_clock[0])
        rn6.append(RTC.sets[:])
    finally:
        gl.st.update(_rn_st)
        gl._ntp[:] = _rn_ntp
        gl._rtc_set[0] = False
        RTC.sets[:] = []
        diag_clear()
    check("RN-6: no fix since boot: the clock is set from NTP's answer once "
          "(clock set ... ntp), a fix later sets it again, a stale answer not",
          rn6 == [[(2026, 10, 31, wd, 12, 0, 5, 0)],
                  ["clock set 2026-10-31T12:00:05Z ntp"],
                  [(2026, 10, 31, wd, 12, 0, 25, 0)], []], repr(rn6))
    # RN-7: Accept-Encoding read like Host (a bare-LF head too), a gzip
    # token with q=0 is no gzip (the bare-LF client got the page inflated,
    # ~5 KB of heap at a bad moment)
    rn7 = [gl._web_gzip(r) for r in (
        b"GET / HTTP/1.1\nHost: x\nAccept-Encoding: gzip\n\n",
        b"GET / HTTP/1.1\r\nAccept-Encoding: deflate, gzip;q=0.5\r\n\r\n",
        b"GET / HTTP/1.1\r\nAccept-Encoding: gzip;q=0\r\n\r\n",
        b"GET / HTTP/1.1\r\nAccept-Encoding: gzip; q=0.000, deflate\r\n\r\n",
        b"GET / HTTP/1.1\r\nAccept-Encoding: identity\r\nX-Gzip: gzip\r\n\r\n",
        b"GET /?gzip HTTP/1.1\r\n\r\n")]
    check("RN-7: gzip in Accept-Encoding in a bare-LF head too, q=0 refuses it",
          rn7 == [True, True, False, False, False, False], repr(rn7))
    # RN-10: the cleanups: wifi_reset's down_* keys, a non-UTF-8 flag value
    # is no flag, _wifi_off takes the wake, the page's ages
    gl.wifi_reset()

    class _RnWlan(object):
        def disconnect(self):
            pass

        def active(self, on=None):
            return False
    rn10 = [gl.wifi["down_home"], gl.wifi["down_ssid"], gl.wifi["down_at"] is not None,
            gl._truthy(b"\xff1"), gl._truthy("1 # on")]
    gl._wifi_off(_RnWlan(), "sleep", 12345)
    rn10 += [gl.wifi["state"], gl.wifi["wake_at"]]
    gl.wifi_reset()
    _rn_rx = gl.st["rx"]
    gl.st["rx"] = _clock[0] - 3000
    rn10.append(dict(gl._web_rows())["rx_age_s"])
    gl.st["rx"] = _rn_rx
    check("RN-10: wifi_reset's down_*, _truthy on bytes, _wifi_off's wake_at, "
          "the page's ages",
          rn10 == [True, None, True, None, True, "sleep", 12345, 3], repr(rn10))
finally:
    (gl.esp32, gl.HTTP_GET, gl.OTA_URL, gl.OTA_KEY, gl.FW_BUILD,
     gl.FW_COMMIT, gl.OTA_PENDING, gl.OTA_BAD, gl.machine.reset,
     gl.DIAG, gl.OTA_TRIES, gl.OTA_PAGES, gl.OTA_MARK, gl.OTA_SHADOW,
     gl._mem_reboot, gl._mem_reboot_soon, _ota_dir) = _rn_saved
    _gm_zero()
    gl._tick_bug.clear()
    gl.ota["pend"] = None
    gl.wifi_reset()
    gl.log_reset()
    gl.up_reset()
    diag_clear()
    shutil.rmtree(_rn_dir, ignore_errors=True)

# --- review 2026-11-08b: net/OTA (RN2-n) ------------------------------------
_n2_dir = tempfile.mkdtemp(prefix="rn2-")
_n2_saved = (gl.esp32, gl.HTTP_GET, gl.OTA_URL, gl.OTA_KEY, gl.FW_BUILD,
             gl.FW_COMMIT, gl.OTA_PENDING, gl.OTA_BAD, gl.machine.reset,
             gl.DIAG, gl.OTA_TRIES, gl.OTA_PAGES, gl.OTA_MARK, gl.OTA_SHADOW,
             gl._mem_reboot, gl._mem_reboot_soon, _ota_dir, gl._diag_write,
             gl.NTP_SERVER, gl._ntp_ask, gl.up["url"], gl._wifi_begin_round,
             gl._try_roam_home, gl.socket, gl.read_uart, gl.feed,
             gl._rtc_sync, gl._tls_wrap, gl._ota_promote, gl._rm,
             gl.wifi["wlan"])
_n2_log = dict((k, gl.log.get(k)) for k in ("clock",))
_n2_ntp = list(gl._ntp)
_n2_st = dict((k, gl.st.get(k)) for k in ("valid", "date", "time", "lat",
                                          "pos_at"))
_ota_dir = _n2_dir
gl.esp32, gl.HTTP_GET = _FakeEsp32, _ota_get
gl.OTA_URL, gl.OTA_KEY = "http://ota.lan/gps/manifest.json", b"secret"
gl.FW_BUILD, gl.FW_COMMIT = 100, "abc0100"
for _k in ("PENDING", "BAD", "TRIES", "MARK"):
    setattr(gl, "OTA_" + _k, os.path.join(_n2_dir, "ota." + _k.lower()))
_n2_page = os.path.join(_n2_dir, "web.html.gz")
gl.OTA_PAGES = (_n2_page,)
gl.OTA_SHADOW = (os.path.join(_n2_dir, "gps_logger.mpy"),)
gl.machine.reset = lambda: _ota_log.append(("reset",))
gl.DIAG = 60000
_n2_reb = []
gl._mem_reboot = lambda why: _n2_reb.append(why)
gl._mem_reboot_soon = lambda: False


def _n2_oom(*a):
    raise MemoryError("memory allocation failed")


def _n2_pending(build=100):
    """A pending image as _ota_check leaves it, health due."""
    _ota_reset()
    with open(_n2_page + ".new", "wb") as f:
        f.write(_page)
    with open(gl.OTA_PENDING, "w") as f:
        f.write("%d abc%04d\n%s" % (build, build, _n2_page))
    with open(gl.OTA_TRIES, "w") as f:
        f.write("%d 1\n" % build)
    with open(gl.OTA_MARK, "w") as f:
        f.write("stop")
    with open(gl.OTA_SHADOW[0] + ".ota-old", "w") as f:
        f.write("old module")
    gl.wifi["state"] = "connected"
    gl.ota.update(pend=build, pend_at=_clock[0] - gl.OTA_VALID_AFTER - 1,
                  wifi_seen=True, probed=True, paths=[_n2_page],
                  state="pending")


def _n2_flash():
    """What is left on flash of a pending image, as a sorted list."""
    return sorted(os.path.basename(f) for f in (
        gl.OTA_PENDING, gl.OTA_TRIES, gl.OTA_MARK, _n2_page + ".new",
        gl.OTA_SHADOW[0] + ".ota-old", _n2_page) if os.path.exists(f))


try:
    # RN2-1: a tick's MemoryErrors count within GUARD_MEM_QUIET of each
    # other: a pass that runs through between them (an upload in its
    # backoff, an idle WiFi or OTA tick) clears nothing, so a lasting one
    # reaches the memory reboot; ones further apart than the window are
    # sporadic and never add up (a page, status or diag tick once a day)
    gl.wifi_reset()
    gl.ota["pend"] = None
    gl._tick_bug.clear()
    _gm_zero()
    del _n2_reb[:]
    for _ in range(gl.GUARD_MEM_MAX):
        _clock[0] += gl.TICK_PAUSE
        gl._guarded("upload", _n2_oom)
        _clock[0] += gl.TICK_PAUSE
        gl._guarded("upload", lambda: None)
    n2_1 = [list(_n2_reb), gl._guard_mem["upload"][0]]
    del _n2_reb[:]
    _gm_zero()
    for name in ("status", "web", "wifi"):
        for _ in range(3 * gl.GUARD_MEM_MAX):
            _clock[0] += gl.GUARD_MEM_QUIET + 1000
            gl._guarded(name, _n2_oom)
            gl._guarded(name, lambda: None)
    n2_1 += [list(_n2_reb), gl._guard_mem["status"][0]]
    check("RN2-1: a tick's guarded MemoryErrors count within GUARD_MEM_QUIET "
          "of each other, a pass between them clears nothing: GUARD_MEM_MAX "
          "reboot; further apart they never add up",
          n2_1 == [["guarded memory upload"], 0, [], 1], repr(n2_1))

    # RN2-2: an image pending: a MemoryError is gc + TICK_PAUSE and counted;
    # GUARD_PEND_MAX of one tick raise (the build crashes, is rolled back
    # and marked bad: a persistent one is its defect); no memory reboot;
    # sporadic ones (further apart than the window) never raise
    gl.ota["pend"] = 200
    gl._tick_bug.clear()
    _gm_zero()
    del _n2_reb[:]
    n2_2 = []
    for i in range(gl.GUARD_PEND_MAX):
        _clock[0] += gl.TICK_PAUSE
        try:
            gl._guarded("upload", _n2_oom)
            n2_2.append("swallowed")
        except MemoryError:
            n2_2.append("raised")
    n2_2.append(list(_n2_reb))
    _gm_zero()
    gl._tick_bug.clear()
    r_ = 0
    for i in range(3 * gl.GUARD_PEND_MAX):
        _clock[0] += gl.GUARD_MEM_QUIET + 1000
        try:
            gl._guarded("upload", _n2_oom)
        except MemoryError:
            r_ += 1
    n2_2.append(r_)
    gl.ota["pend"] = None
    check("RN2-2: with an image pending a MemoryError is swallowed and "
          "counted, GUARD_PEND_MAX of one tick raise (no memory reboot); "
          "sporadic ones never do",
          n2_2 == ["swallowed"] * (gl.GUARD_PEND_MAX - 1) + ["raised", [], 0],
          repr(n2_2))
    gl._tick_bug.clear()
    _gm_zero()

    # RN2-3: the pending state ends after the confirmation steps. A
    # MemoryError in _ota_promote (swallowed by the guard) left the RAM
    # state "done" over the note, the staged page, OTA_TRIES and .ota-old
    # on flash, and ota_tick never came back to finish; now the next tick
    # runs the steps again. One in the closing diag line changes nothing
    _real_promote = gl._ota_promote
    n2_boom = [1]

    def _n2_promote(paths):
        if n2_boom[0]:
            n2_boom[0] = 0
            raise MemoryError("m")
        _real_promote(paths)
    gl._ota_promote = _n2_promote
    _n2_pending()
    gl._tick_bug.clear()
    gl._guarded("ota", gl.ota_tick, _clock[0])
    n2_3 = [gl.ota["pend"], gl.ota["state"], _n2_flash()]
    _clock[0] += gl.TICK_PAUSE
    gl._guarded("ota", gl.ota_tick, _clock[0])
    with open(_n2_page, "rb") as f:
        n2_3 += [gl.ota["pend"], gl.ota["state"], _n2_flash(), f.read() == _page]
    gl._ota_promote = _real_promote
    _n2_pending()
    _real_dw = gl._diag_write
    gl._diag_write = lambda line: (_ for _ in ()).throw(MemoryError("m"))
    gl._tick_bug.clear()
    try:
        gl._guarded("ota", gl.ota_tick, _clock[0])
    finally:
        gl._diag_write = _real_dw
    n2_3 += [gl.ota["pend"], gl.ota["state"], _n2_flash()]
    # run again (a retry): the page is up already, no "not put up" line
    diag_clear()
    gl._ota_promote([_n2_page])
    n2_3.append([l for l in diag_lines() if "not put up" in l])
    check("RN2-3: _ota_health ends the pending state after the steps: a "
          "MemoryError in one leaves it (and the flash) as it was, the next "
          "tick finishes; in the closing diag line nothing is left over; "
          "_ota_promote can run twice",
          n2_3 == [100, "pending",
                   ["gps_logger.mpy.ota-old", "ota.mark", "ota.pending",
                    "ota.tries", "web.html.gz.new"],
                   None, "ok", ["web.html.gz"], True,
                   None, "ok", ["web.html.gz"], []], repr(n2_3))

    # RN2-4: the clock a fix gave this boot sets the board clock when the
    # fix is gone and no POST has set it yet (it was NTP's, or nothing: a
    # board that had a fix outdoors, indoors at home, failed every verified
    # POST's dates); it wins over NTP's answer, a stale one sets nothing
    gl.st.update(valid=False)
    wd = _dt.date(2026, 10, 31).weekday()
    secs = ((_dt.date(2026, 10, 31) - _dt.date(2020, 1, 1)).days * 86400
            + 12 * 3600)
    RTC.dt, RTC.sets[:] = (2000, 1, 1, 5, 0, 0, 0, 0), []
    gl._rtc_set[0] = False
    diag_clear()
    gl._ntp[0] = (secs + 3600, _clock[0] - 1000)
    gl.log["clock"] = (secs, _clock[0] - 30000)
    gl._rtc_sync(_clock[0])
    n2_4 = [RTC.sets[:], [l for l in diag_lines() if l.startswith("clock")]]
    RTC.sets[:] = []
    gl._rtc_set[0] = False
    gl.log["clock"] = (secs, _clock[0] - gl.TICKS_OLD - 10)
    gl._ntp[0] = None
    gl._rtc_sync(_clock[0])
    n2_4.append(RTC.sets[:])
    check("RN2-4: no fix now, none set the board clock yet: the last fix's "
          "clock run on by the ticks sets it (over NTP's answer), a stale "
          "one does not",
          n2_4 == [[(2026, 10, 31, wd, 12, 0, 30, 0)],
                   ["clock set 2026-10-31T12:00:30Z gps"], []], repr(n2_4))
    gl._rtc_set[0] = False
    RTC.sets[:] = []
    diag_clear()

    # RN2-5: a fix told the time, the board clock is not set yet and https
    # is in use (GEO_URL or OTA_URL): NTP is asked on (it was never again
    # after a fix); an http board, or one with the clock set, is not asked
    asked = []
    gl._ntp_ask = lambda: asked.append(1)
    gl.NTP_SERVER = "ntp.test"
    gl.wifi["state"] = "connected"
    gl.log["clock"] = (secs, _clock[0])
    n2_5 = []
    for url, ota_url, rtc_set, clk in (
            ("https://d.example", "http://ota.lan/m.json", False, True),
            ("http://d.example", "http://ota.lan/m.json", False, True),
            ("http://d.example", "https://ota.lan/m.json", False, True),
            ("https://d.example", "https://ota.lan/m.json", True, True),
            ("http://d.example", "http://ota.lan/m.json", False, False)):
        gl.up["url"], gl.OTA_URL = url, ota_url
        gl._rtc_set[0] = rtc_set
        gl.log["clock"] = (secs, _clock[0]) if clk else None
        gl._ntp[:] = [None, None, gl.NTP_RETRY]
        del asked[:]
        gl.ntp_tick(_clock[0])
        n2_5.append(len(asked))
    check("RN2-5: after a fix NTP is asked while the board clock is not set "
          "and https is used (GEO_URL or OTA_URL); not for http, not once "
          "set; before a fix as before",
          n2_5 == [1, 0, 1, 0, 1], repr(n2_5))
    gl.OTA_URL = "http://ota.lan/gps/manifest.json"
    gl._rtc_set[0] = False

    # RN2-6: an https GET (the OTA's) sets the board clock first, as a POST
    # does; an http one does not
    class _N2Sock(_GetSock):
        ev = []
        def connect(self, a):
            _N2Sock.ev.append("connect")
    gl.socket, gl.read_uart = _gsock, lambda: None
    gl._rtc_sync = lambda now: _N2Sock.ev.append("rtc")
    gl._tls_wrap = lambda s, h: s
    n2_6 = []
    try:
        for u in ("https://h/x", "http://h/x"):
            del _N2Sock.ev[:]
            _gs[0] = _N2Sock(b"HTTP/1.0 404 Not Found\r\n\r\n")
            gl._raw_get(u, bytearray().extend, 100)
            n2_6.append(list(_N2Sock.ev))
        # RN2-8 (the answers): no status line, one token, a long head
        n2_8 = []
        for resp in (b"", b"HTTP/1.0\r\n\r\n"):
            _gs[0] = _GetSock(resp)
            try:
                gl._raw_get("http://h/x", bytearray().extend, 100)
                n2_8.append("ok")
            except OSError:
                n2_8.append("OSError")
        hdrs = b"".join(b"X-Pad-%d: y\r\n" % i for i in range(gl.HTTP_LINES + 6))
        _gs[0] = _GetSock(b"HTTP/1.0 404 Not Found\r\n" + hdrs + b"\r\n")
        n2_8.append(gl._raw_get("http://h/x", bytearray().extend, 100))
        # the DNS lookup is fed around
        fed = []
        gl.feed = lambda: fed.append("feed")
        _gsock.getaddrinfo = lambda h, p, a=0, b=0: (
            fed.append("dns") or [(2, 1, 6, "", (h, p))])
        _gs[0] = _GetSock(b"HTTP/1.0 404 Not Found\r\n\r\n")
        gl._http_open("GET", "http://h/x")
        n2_6.append(fed)
    finally:
        gl.feed = _n2_saved[25]
        gl._rtc_sync = _n2_saved[26]
        gl._tls_wrap = _n2_saved[27]
        gl.socket, gl.read_uart = _n2_saved[23], _n2_saved[24]
    check("RN2-6: an https GET sets the board clock before it connects, an "
          "http one not; getaddrinfo is fed around",
          n2_6 == [["rtc", "connect"], ["connect"], ["feed", "dns", "feed"]],
          repr(n2_6))
    # RN2-8: _raw_get: a closed connection or a short status line is an
    # OSError (it was an IndexError, outside what _ota_check noted), a non-200
    # answer with a head over HTTP_LINES pieces still gives its status
    gl.HTTP_GET = _ota_get
    _ota_reset()
    _serve(_manifest(200))
    n2_dl = []
    for exc in (IndexError("x"), MemoryError("m"), OSError(104)):
        _ota_reset()
        _serve(_manifest(200))

        def _n2_get(url, sink, limit, exc=exc):
            if url.endswith("micropython.bin"):
                raise exc
            return _ota_get(url, sink, limit)
        gl.HTTP_GET = _n2_get
        try:
            gl._ota_check()
        except Exception:
            pass
        f_ = gl.ota.get("failed")
        n2_dl.append(f_ and f_[0::2])
    gl.HTTP_GET = _ota_get
    check("RN2-8: _raw_get gives an OSError for no or a short status line "
          "and the status of a non-200 answer with a long head; a download "
          "that raises anything is noted as failed (OTA_RETRY)",
          n2_8 == ["OSError", "OSError", 404]
          and n2_dl == [(200, "download err")] * 3, repr((n2_8, n2_dl)))

    # RN2-7: an unreadable /ota.pending: its build is not known (this
    # image or the old one after a rollback). Held as the running build's it
    # kept the radio on, blocked the memory reboots and, away from WiFi,
    # reset after OTA_HEALTH and counted a try (or a bad mark) against a
    # build that may be the good old one, for ever if the note would not go.
    # Dropped, with the staged page, and the running image confirmed once
    n2_7 = []
    for fwb in (200, 100):
        _ota_reset()
        gl.FW_BUILD = fwb
        with open(gl.OTA_PENDING, "wb") as f:
            f.write(b"\xff\xfe garbage\n")
        with open(gl.OTA_MARK, "w") as f:
            f.write("crash")
        with open(_n2_page + ".new", "wb") as f:
            f.write(_page)
        diag_clear()
        gl.ota_boot()
        n2_7.append((gl.ota["pend"], _n2_flash(), ("valid",) in _ota_log,
                     os.path.exists(gl.OTA_TRIES), os.path.exists(gl.OTA_BAD),
                     gl._ota_wants_radio(),
                     [l for l in diag_lines() if "unreadable" in l]))
    # the note cannot be removed: still no pending state, no reset
    _ota_reset()
    with open(gl.OTA_PENDING, "wb") as f:
        f.write(b"\xff\xfe garbage\n")
    gl._rm = lambda path: None
    try:
        gl.ota_boot()
        gl.ota_tick(_clock[0] + gl.OTA_HEALTH * 2)
    finally:
        gl._rm = _n2_saved[29]
    n2_7.append((gl.ota["pend"], ("reset",) in _ota_log, gl._ota_wants_radio()))
    gl.FW_BUILD = 100
    check("RN2-7: an unreadable /ota.pending is dropped with the staged page "
          "and the running image confirmed once: no pending state, no radio "
          "held, no try or bad mark, no reset even if it cannot be removed",
          n2_7 == [(None, [], True, False, False, False,
                    ["ota note unreadable, dropped (running abc0100)"])] * 2
          + [(None, False, False)], repr(n2_7))
    _ota_reset()
    gl.ota["pend"] = None

    # RN2-9: gps_boot and the zero tick. unshadow: a remove of the stale
    # .ota-old that fails must not fall into the rename that puts it over
    # the module deployed since; crash(): only a note naming the running
    # build is blamed (the old firmware failing after a rollback is not the
    # new build's fault, nor is an unreadable note); a tick of 0 is a time
    import gps_boot as gb
    gb_saved = (gb.OTA_PENDING, gb.OTA_MARK, gb.OTA_SHADOW,
                sys.modules.get("gps_frozen"), os.remove)
    n2_9 = []
    try:
        gb.OTA_PENDING, gb.OTA_MARK = gl.OTA_PENDING, gl.OTA_MARK
        gb.OTA_SHADOW = (os.path.join(_n2_dir, "m.mpy"),)
        fz = sys.modules["gps_frozen"] = types.ModuleType("gps_frozen")
        fz.BUILD = 100
        _ota_reset()
        with open(gb.OTA_PENDING, "w") as f:
            f.write("200 abc0200\n")
        for fresh in (True, False):
            for q in (gb.OTA_SHADOW[0], gb.OTA_SHADOW[0] + ".ota-old"):
                try:
                    os.remove(q)
                except OSError:
                    pass
            with open(gb.OTA_SHADOW[0] + ".ota-old", "w") as f:
                f.write("stale")
            if fresh:
                with open(gb.OTA_SHADOW[0], "w") as f:
                    f.write("fresh")
            real_rm = os.remove

            def _n2_rm(path, real_rm=real_rm):
                if path.endswith(".ota-old"):
                    raise OSError(5, "EIO")
                real_rm(path)
            os.remove = _n2_rm
            try:
                gb.unshadow()
            finally:
                os.remove = real_rm
            with open(gb.OTA_SHADOW[0]) as f:
                n2_9.append(f.read())
        # crash(): note of the running build, of another, unreadable
        for note in (b"100 abc0100\n", b"200 abc0200\n", b"\xff garbage"):
            _ota_reset()
            with open(gb.OTA_PENDING, "wb") as f:
                f.write(note)
            gb.crash()
            n2_9.append(gl._ota_mark_get())
    finally:
        (gb.OTA_PENDING, gb.OTA_MARK, gb.OTA_SHADOW, fzm, os.remove) = gb_saved
        if fzm is None:
            del sys.modules["gps_frozen"]
        else:
            sys.modules["gps_frozen"] = fzm
    check("RN2-9: unshadow keeps the module deployed since when the remove "
          "of .ota-old fails (and puts it back when there is none); crash() "
          "blames the build only when the note names the running one",
          n2_9 == ["fresh", "stale", "crash", None, None], repr(n2_9))

    class _N2W(object):
        def status(self, *a):
            return gl.network.STAT_CONNECTING
        def disconnect(self):
            pass
        def active(self, *a):
            return True
    gl.wifi_reset()
    gl.wifi.update(wlan=_N2W(), state="lost", try_at=0, home=False)
    gl._wifi_begin_round = lambda w: gl.wifi.update(state="round")
    _clock[0] = max(_clock[0], gl.LOST_GRACE + 5000)
    gl.wifi_tick()
    n2_0 = [gl.wifi["state"]]
    # connected away from home, roam_at 0, due: the roam is tried
    called = []
    gl._try_roam_home = lambda w, arrival=False: called.append(1)
    gl.wifi_reset()
    gl.wifi.update(wlan=_N2W(), state="connected", home=False, roam_at=0,
                   nets=[{"ssid": "h", "pw": "", "home": True,
                          "hidden": False, "bssid": None, "ip": None}],
                   idle_at=_clock[0], last_check=_clock[0])
    _N2W.status = lambda self, *a: gl.network.STAT_GOT_IP
    gl.wifi_tick()
    n2_0.append(len(called))
    check("RN2-9b: a tick of 0 is a time: a lost link past LOST_GRACE gives "
          "up, a roam due from roam_at 0 is tried (both read 0 as unset)",
          n2_0 == ["round", 1], repr(n2_0))
finally:
    (gl.esp32, gl.HTTP_GET, gl.OTA_URL, gl.OTA_KEY, gl.FW_BUILD,
     gl.FW_COMMIT, gl.OTA_PENDING, gl.OTA_BAD, gl.machine.reset,
     gl.DIAG, gl.OTA_TRIES, gl.OTA_PAGES, gl.OTA_MARK, gl.OTA_SHADOW,
     gl._mem_reboot, gl._mem_reboot_soon, _ota_dir, gl._diag_write,
     gl.NTP_SERVER, gl._ntp_ask, gl.up["url"], gl._wifi_begin_round,
     gl._try_roam_home, gl.socket, gl.read_uart, gl.feed,
     gl._rtc_sync, gl._tls_wrap, gl._ota_promote, gl._rm,
     gl.wifi["wlan"]) = _n2_saved
    gl.log.update(_n2_log)
    gl._ntp[:] = _n2_ntp
    gl.st.update(_n2_st)
    gl._rtc_set[0] = False
    RTC.sets[:] = []
    _gm_zero()
    gl._tick_bug.clear()
    gl.ota["pend"] = None
    gl.wifi_reset()
    gl.log_reset()
    gl.up_reset()
    diag_clear()
    shutil.rmtree(_n2_dir, ignore_errors=True)

print()
print("%d passed, %d failed" % (passed, failed))
if failures:
    print("failures:")
    for f in failures:
        print(" ", f)
sys.exit(1 if failed else 0)
