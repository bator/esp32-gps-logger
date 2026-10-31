# The status page over real loopback sockets, on the MicroPython 1.29 unix
# port (and CPython): web_tick / _web_read / _web_send against clients that
# are slow, send nothing, send too much, reset mid-reply or outlast the
# request deadline. Single-threaded: a client's bytes wait in the kernel
# until web_tick reads them.
# Run: /path/to/micropython tests/web_loopback.py [web.html.gz]  (or python3)
# With a gzip of web.html as the argument, a client that accepts gzip must
# get that file (tests/test_gps_logger.py passes one).
import sys
import json
import time
import gc
import socket
import struct
import _thread

here = __file__
i = here.rfind("/")
script_dir = here[:i] if i >= 0 else "."
i = script_dir.rfind("/")
root = script_dir[:i] if i >= 0 else "."
sys.path.insert(0, root)


class _Obj:
    pass


class _Dummy:
    def __init__(self, *a, **k):
        pass

    def __call__(self, *a, **k):
        return self

    def __getattr__(self, name):
        return _Dummy()


class _Uart(_Dummy):
    def any(self):
        return 0

    def read(self, n=None):
        return None

    def write(self, b):
        return len(b)


machine = _Obj()
machine.Pin = machine.I2C = machine.WDT = machine.Timer = _Dummy
machine.RTC = _Dummy
machine.UART = _Uart
machine.freq = lambda *a: 80000000
machine.reset_cause = lambda: 1
machine.reset = lambda: None
sys.modules["machine"] = machine
network = _Obj()
network.STA_IF = 0
network.STAT_IDLE, network.STAT_CONNECTING, network.STAT_GOT_IP = 0, 1, 3
network.WLAN = _Dummy
network.hostname = lambda *a: "x"
sys.modules["network"] = network

if not hasattr(time, "ticks_ms"):       # CPython
    time.ticks_ms = lambda: int(time.time() * 1000) & 0x3FFFFFFF
    time.ticks_diff = lambda a, b: ((a - b + 0x20000000) & 0x3FFFFFFF) \
        - 0x20000000
    time.ticks_add = lambda a, b: (a + b) & 0x3FFFFFFF
    time.sleep_ms = lambda ms: time.sleep(ms / 1000)
if not hasattr(gc, "threshold"):
    gc.threshold = lambda *a: None
if not hasattr(gc, "mem_alloc"):
    gc.mem_alloc = lambda: 50000
if not hasattr(gc, "mem_free"):
    gc.mem_free = lambda: 100000

import gps_logger as gl                 # noqa: E402

gl.DIAG = 0
gl.DIAG_FILE = "/nonexistent-gps-logger-loopback/diag.log"
gl.LOG_DIR = "/nonexistent-gps-logger-loopback"
gl.WEB_PAGE = root + "/web.html"
gz_path = sys.argv[1] if len(sys.argv) > 1 else None
gl.WEB_PAGE_GZ = gz_path or "/nonexistent-gps-logger-loopback/web.html.gz"
gl.WEB = True
gl.WEB_DEADLINE = 1                     # s, short for the test
gl.WEB_TIMEOUT = 0.4
gl.reset()
gl.log_reset()
gl.wifi_reset()
gl.up_reset()
gl.wifi.update(state="connected", ip="127.0.0.1")
f = open(gl.WEB_PAGE, "rb")
page = f.read()
f.close()
fails = []


def check(name, ok, detail=""):
    print(("ok  " if ok else "FAIL") + " " + name
          + ("" if ok else " - " + str(detail)))
    if not ok:
        fails.append(name)


port = None
for k in range(8):                      # a free port
    gl.WEB_PORT = 20000 + (time.ticks_ms() * 7 + k * 1013) % 20000
    gl.web_tick()
    if gl._web_sock is not None:
        port = gl.WEB_PORT
        break
check("listens on a loopback port", port is not None)
if port is None:
    raise SystemExit("web loopback FAILED: no port")
addr = socket.getaddrinfo("127.0.0.1", port)[0][-1]

outs = []                               # bytes of each send of a request


class Conn:
    """The accepted socket, its sends counted."""
    def __init__(self, c):
        self.c = c

    def settimeout(self, t):
        self.c.settimeout(t)

    def recv(self, n):
        return self.c.recv(n)

    def sendall(self, b):
        outs.append(len(b))
        self.c.sendall(b)

    def close(self):
        self.c.close()


class Listen:
    def __init__(self, s):
        self.s = s

    def accept(self):
        c, a = self.s.accept()
        return Conn(c), a

    def close(self):
        self.s.close()


gl._web_sock = Listen(gl._web_sock)


def client(data=b""):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(addr)
    if data:
        s.send(data)
    return s


def read_all(s):
    s.settimeout(2)
    got = b""
    try:
        while True:
            b = s.recv(4096)
            if not b:
                break
            got += b
    except OSError:                     # a reset after the reply
        pass
    s.close()
    return got


_served = [False]
heads = []                              # what _web_read returned
real_read = gl._web_read


def tracking_read(c):
    _served[0] = True
    heads.append(real_read(c))
    return heads[-1]


gl._web_read = tracking_read


def tick():
    """web_tick until it has taken the waiting client; ms that took."""
    outs[:] = []
    _served[0] = False
    t0 = time.ticks_ms()
    for _ in range(100):
        gl.web_tick()
        if _served[0]:
            break
        time.sleep_ms(10)
    return time.ticks_diff(time.ticks_ms(), t0)


req = b"GET / HTTP/1.1\r\nHost: 127.0.0.1:%d\r\n\r\n" % port

# a normal page and the JSON
s = client(req)
tick()
got = read_all(s)
check("GET / sends web.html whole", got.split(b"\r\n\r\n", 1)[-1] == page
      and got.startswith(b"HTTP/1.0 200 OK"), got[:80])
if gz_path:
    f = open(gz_path, "rb")
    gz_page = f.read()
    f.close()
    s = client(b"GET / HTTP/1.1\r\nHost: 127.0.0.1:%d\r\n"
               b"Accept-Encoding: gzip, deflate\r\n\r\n" % port)
    tick()
    got = read_all(s)
    gh, _, gb = got.partition(b"\r\n\r\n")
    check("GET / with Accept-Encoding gzip sends web.html.gz whole",
          gb == gz_page and b"\r\nContent-Encoding: gzip\r\n" in gh
          and (b"Content-Length: %d\r\n" % len(gz_page)) in gh,
          (gh[:200], len(gb), len(gz_page)))
s = client(b"GET /status.json HTTP/1.1\r\nHost: localhost\r\n\r\n")
tick()
got = read_all(s)
check("a foreign Host gets 403", got.startswith(b"HTTP/1.0 403"), got[:80])
s = client(b"GET /status.json HTTP/1.0\r\n\r\n")
tick()
got = read_all(s)
try:
    jd = json.loads(got.split(b"\r\n\r\n", 1)[1])
except Exception:
    jd = None
check("GET /status.json is valid JSON", jd is not None and "fix" in jd,
      got[:120])
check("no send over WEB_CHUNK", max(outs) <= gl.WEB_CHUNK, outs)

# empty request: connect and close
s = client()
s.close()
ms = tick()
check("an empty request is closed unanswered, at once",
      outs == [] and heads[-1] == b"" and ms < 1000, (outs, heads[-1], ms))

# oversized head: no blank line within WEB_REQ_MAX
s = client(b"GET / HTTP/1.1\r\nX-Pad: " + b"a" * 4000)
ms = tick()
read_all(s)
check("an oversized head is read only up to WEB_REQ_MAX(+512)",
      gl.WEB_REQ_MAX <= len(heads[-1]) < gl.WEB_REQ_MAX + 512
      and ms < 1500, (len(heads[-1]), ms))

# stalled client: half a request line, then nothing
s = client(b"GET / HT")
ms = tick()
check("a stalled request is dropped after WEB_TIMEOUT, unanswered",
      outs == [] and 300 <= ms <= 900, (outs, ms))
s.close()

# slow client: a byte every 100 ms (each recv in time, the request never
# done) from a thread
trickle_on = [True]


def trickle(sock):
    try:
        for b in b"GET / HTTP/1.1\r\nX-Slow: " + b"a" * 100:
            if not trickle_on[0]:
                break
            sock.send(bytes((b,)))
            time.sleep_ms(100)
    except OSError:
        pass


s = client()
_thread.start_new_thread(trickle, (s,))
ms = tick()
trickle_on[0] = False
time.sleep_ms(150)
s.close()
check("a trickling request is dropped at WEB_DEADLINE, unanswered",
      outs == [] and 900 <= ms <= 1600, (outs, ms))

# a client that resets the connection mid-reply
real_uart = gl.read_uart
victim = [None]


def reset_client():
    if victim[0] is not None and len(outs) >= 2:
        v = victim[0]
        victim[0] = None
        v.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER,
                     struct.pack("ii", 1, 0))
        v.close()                       # RST
        time.sleep_ms(50)


gl.read_uart = reset_client
s = client(req)
victim[0] = s
crash = None
try:
    tick()
except Exception as e:
    crash = e
gl.read_uart = real_uart
check("a reset mid-reply ends the reply, no raise",
      crash is None and victim[0] is None
      and sum(outs) < len(page), (crash, len(outs), sum(outs)))

# reply slower than the deadline (a slow link: every chunk 150 ms)
gl.read_uart = lambda: time.sleep_ms(150)
s = client(req)
ms = tick()
gl.read_uart = real_uart
got = read_all(s)
check("a reply past WEB_DEADLINE is cut there",
      ms <= 1600 and len(got) < len(page), (ms, len(got)))

# and the server still answers afterwards
s = client(req)
tick()
got = read_all(s)
check("still answering after all that", got.endswith(page), len(got))

gl._web_close()
if fails:
    raise SystemExit("web loopback FAILED: %s" % ", ".join(fails))
print("web loopback passed")
