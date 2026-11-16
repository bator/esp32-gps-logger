"""A stateful UC6580 and the loop around gps_receiver.py, on (f32) unix
MicroPython: a fake receiver (answers its setup, 1 / 5 Hz, GST, restarts with
and without a banner, silences of a minute to days, fix-less spells, lost
and cut sentences) on an 8 KB UART ring, the loop's reads and gps_check,
blocks with and without the POST guard's reads, all on a fake clock (ticks
wrap at 2**30). After each pass every published second is checked against
what the receiver sent: no fix dated with a day it was not sent on, no
position out of range, no exception, no bug; at the end, after 20 quiet
minutes, the setup must be back (5 Hz with GST).

    micropython -X heapsize=64M tests/rcv_sim.py SEED [EVENTS [x START_TICKS]]

Also: silences of GPS_SILENT +- 1.5 s, blocks of 60-119 s with no read while
the receiver sends on, a restart right at a silence's end, a clock that
starts near the ticks wrap (START_TICKS); long silences in 5 s passes.

One line "seed N bad 0 ..." when it holds. Not deployed."""
import sys
class _T: pass
T = _T()
clk = [int(sys.argv[4]) if len(sys.argv) > 4 else 1000]
T.ticks_ms = lambda: clk[0] & 0x3FFFFFFF
T.ticks_diff = lambda a, b: ((a - b + 0x20000000) & 0x3FFFFFFF) - 0x20000000
T.ticks_add = lambda a, b: (a + b) & 0x3FFFFFFF
def _sl(ms):
    clk[0] += ms
T.sleep_ms = _sl
T.sleep = lambda s: _sl(int(s * 1000))
T.time = lambda: clk[0] // 1000
sys.modules["time"] = T
_d = __file__.rsplit("/", 1)[0] if "/" in __file__ else "."   # tests/
if _d in (".", ""):
    _ROOT = ".."                         # (run from tests/)
elif "/" in _d:
    _ROOT = _d.rsplit("/", 1)[0] or "/"
else:
    _ROOT = "."                          # (run from the repo: tests/rcv_sim.py)
sys.path.insert(0, _ROOT)
import random

def cs(body):
    c = 0
    for ch in body: c ^= ord(ch)
    return "$%s*%02X\r\n" % (body, c)

import gps_clock as C
DIAG = []

class Rx:
    """The fake receiver: emits NMEA by sim time into a UART ring of RXBUF."""
    RXBUF = 8192
    def __init__(self, utc0):
        self.utc0 = utc0                 # UTC s since 2020 at clk 0
        self.buf = bytearray()
        self.defaults()
        self.next_ep = None              # next epoch (ms of clk)
        self.silent_until = 0
        self.fixless_until = 0
        self.ignore = False              # answers but ignores
        self.mute_cmd = False            # does not answer
        self.emitted = bytearray(40 * 86400 // 8 + 1)
        self.drop = 0.0                  # chance a sentence is lost
        self.rmc_drop = 0.0
        self.cut = 0.0
        self.out = []
        self.pending = []                # replies (at ms, bytes)
        self.restarts = 0
    def defaults(self):
        self.hz = 1; self.gst = 0; self.gsa = 1; self.gsv = 1; self.txt = True
    def utc_ms(self, ms):
        return self.utc0 * 1000 + ms
    def stamp(self, ms):
        u = self.utc_ms(ms)
        s = u // 1000
        y, m, d = C.civil(s // 86400 + C.EPOCH_DAYS)
        r = s % 86400
        frac = (u % 1000) // 10
        t = "%02d%02d%02d.%02d" % (r // 3600, r // 60 % 60, r % 60, frac)
        return t, "%02d%02d%02d" % (d, m, y % 100), "%04d-%02d-%02d" % (y, m, d), "%02d:%02d:%02d" % (r // 3600, r // 60 % 60, r % 60)
    def put(self, s):
        if random.random() < self.drop:
            return
        b = s.encode() if isinstance(s, str) else s
        if random.random() < self.cut:
            b = b[:random.randint(0, len(b) - 1)]
        room = self.RXBUF - len(self.buf)
        if room <= 0:
            return
        self.buf.extend(b[:room])
    def restart(self, ms):
        self.restarts += 1
        self.defaults()
        self.fixless_until = ms + random.choice([0, 1000, 5000, 30000])
        self.put("\r\nUC6580 FWVer R3.4.21\r\n")
        self.next_ep = ms + 300
    def pump(self, now):
        while self.pending and self.pending[0][0] <= now:
            self.put(self.pending.pop(0)[1])
        if self.next_ep is None:
            self.next_ep = now
        step = 1000 // self.hz
        while self.next_ep <= now:
            ms = self.next_ep
            # epochs on the receiver's grid
            u = self.utc_ms(ms)
            u -= u % step
            ms0 = u - self.utc0 * 1000
            self.next_ep = ms0 + step
            if ms0 < self.silent_until:
                continue
            self.epoch(ms0)
    def epoch(self, ms):
        t, ddmmyy, date, hms = self.stamp(ms)
        fix = ms >= self.fixless_until
        k = (self.utc_ms(ms) // (1000 // self.hz)) % (self.hz * 5)
        lat = "4730.%05d" % random.randint(0, 99999)
        if fix:
            if random.random() >= self.rmc_drop:
                self.put(cs("GNRMC,%s,A,%s,N,01903.98765,E,%.1f,90.0,%s,,,A,V" % (t, lat, random.choice([0.1, 0.5, 20.0]), ddmmyy)))
            self.put(cs("GNGGA,%s,%s,N,01903.98765,E,1,20,0.8,100.0,M,40.0,M,," % (t, lat)))
            k = self.utc_ms(ms) // 1000 - self.utc0; self.emitted[k >> 3] |= 1 << (k & 7)
        else:
            self.put(cs("GNRMC,%s,V,,,,,,,170126,,,N,V" % t))
            self.put(cs("GNGGA,%s,,,,,0,00,99.9,,,,,," % t))
        if self.gst and k % (self.hz * 1) == 0:
            self.put(cs("GNGST,%s,1.0,2.0,1.5,30.0,1.2,1.3,2.5" % t))
        if k == 0:
            for sysid in (1, 4):
                self.put(cs("GNGSA,A,3,01,02,03,04,05,06,07,08,09,10,11,12,1.2,0.8,0.9,%d" % sysid))
            self.put(cs("GPGSV,1,1,04,01,45,120,40,02,30,200,35,03,10,300,20,04,60,050,45,1"))
            if self.txt:
                self.put(cs("GPTXT,01,01,02,ANTENNA OK"))
    def write(self, b, now):
        s = bytes(b).decode()
        self.out.append(s)
        if len(self.out) > 64:
            del self.out[:32]            # (the commands seen lately only)
        if self.mute_cmd or now < self.silent_until:
            return
        cmd = s[1:].split("*")[0].strip()
        rep = "$OK*04\r\n"
        if cmd == "PDTINFO":
            rep = cs("PDTINFO,UC6580,R3.4,x,y,z") + rep
        elif not self.ignore:
            if cmd.startswith("CFGNAV,200"):
                self.hz = 5
            elif cmd == "CFGMSG,0,7,5":
                self.gst = 1
            elif cmd in ("CFGMSG,6,0,0",):
                self.txt = False
        self.pending.append((now + 11, rep))

class Uart:
    def __init__(self, rx): self.rx = rx
    def any(self):
        self.rx.pump(clk[0]); return len(self.rx.buf)
    def readinto(self, mv):
        self.rx.pump(clk[0])
        b = self.rx.buf
        n = min(len(mv), len(b))
        mv[0:n] = b[0:n]
        self.rx.buf = b[n:]
        return n
    def write(self, b):
        self.rx.write(b, clk[0]); return len(b)

class _M: pass
machine = _M()
machine.UART = lambda *a, **k: None
sys.modules["machine"] = machine
import gps_receiver as r
r._diag_write = lambda line: DIAG.append((clk[0], line))

def was_emitted(rx, date, hms):
    v = C.dt_s(date, hms)
    if v is None: return False
    k = v - rx.utc0
    return 0 <= k < len(rx.emitted) * 8 and rx.emitted[k >> 3] >> (k & 7) & 1

seed = int(sys.argv[1]) if len(sys.argv) > 1 else 1
N = int(sys.argv[2]) if len(sys.argv) > 2 else 400
VERB = len(sys.argv) > 3 and sys.argv[3] == "v"
random.seed(seed)
# 2026-11-15 23:58:30 UTC
utc0 = (C.days(2026, 11, 15) - C.EPOCH_DAYS) * 86400 + 23 * 3600 + 58 * 60 + 30
rx = Rx(utc0 - clk[0] // 1000)
r.uart = Uart(rx)
r.reset()
r.gps_setup()
seen = [0]
bad = [0]
events = []
def note(*a):
    events.append((clk[0], a))
    if VERB: print(clk[0], *a)
def check():
    for f in r.drain(seen[0]):
        seen[0] = f[r.F_SEQ]
        if f[r.F_VALID]:
            lat, lon = f[r.F_LAT], f[r.F_LON]
            if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                print("RANGE", lat, lon); bad[0] += 1
            if f[r.F_DATE] is not None and not was_emitted(rx, f[r.F_DATE], f[r.F_TIME]):
                print("WRONG DATE", f[r.F_DATE], f[r.F_TIME], "true", rx.stamp(clk[0])[2:], "seed", seed, "events", events[-4:])
                bad[0] += 1
def pass_(dt=None):
    clk[0] += dt if dt is not None else random.randint(20, 60)
    nb = r.st["bug"]
    try:
        r.read_uart()
        r.gps_check(T.ticks_ms())
    except Exception as e:
        print("ESCAPED", repr(e)); bad[0] += 1
        if hasattr(sys, "print_exception"):
            sys.print_exception(e)
    if r.st["bug"] != nb:
        print("BUG", r._bugs); bad[0] += 1
    check()
def run_for(ms, step=None):
    """Passes for ms; step (a long silence's 5 s) only up to 6 s before the
    end, then the loop's own 20-60 ms (5 s passes ran up to 5 s past it: a
    restart "at the silence's end" came seconds after)."""
    end = clk[0] + ms
    while clk[0] < end:
        pass_(step if step is not None and end - clk[0] > step + 6000 else None)
def block(ms, timer):
    end = clk[0] + ms
    while clk[0] < end:
        clk[0] += 100
        if timer:
            r.read_uart()
        check()
for ev in range(N):
    x = random.random()
    rx.rmc_drop = random.choice([0, 0, 0.3, 1.0]) if random.random() < 0.2 else rx.rmc_drop
    rx.drop = random.choice([0, 0, 0.05]) if random.random() < 0.2 else rx.drop
    rx.cut = random.choice([0, 0, 0.05]) if random.random() < 0.2 else rx.cut
    if x < 0.08:
        d = r.GPS_SILENT + random.randint(-1500, 1500); note("silence near", d)
        rx.silent_until = clk[0] + d
        run_for(d + random.randint(0, 3000))
        continue
    if x < 0.14:
        d = random.randint(60000, 119000); note("long block no read", d); block(d, False)
        continue
    if x < 0.20:
        d = random.choice([r.GPS_SILENT + random.randint(-200, 200), random.randint(61000, 3 * 86400000)])
        note("restart at silence end", d)
        rx.silent_until = clk[0] + d
        run_for(d - 1500, 5000 if d > 100000 else None)
        e = rx.silent_until + random.randint(-50, 50)
        while clk[0] < e:
            pass_(random.randint(1, 30))
        rx.restart(clk[0])
        if random.random() < 0.3:
            rx.buf = bytearray()
        continue
    x = random.random()
    if x < 0.30:
        run_for(random.randint(500, 20000))
    elif x < 0.45:
        d = random.randint(500, 60000); note("block timer", d); block(d, True)
    elif x < 0.55:
        d = random.randint(500, 12000); note("block", d); block(d, False)
    elif x < 0.65:
        d = random.choice([61000, 300000, 3600000, 86400000 + random.randint(0, 7200000), 2 * 86400000 + 5000])
        note("silence", d); rx.silent_until = clk[0] + d
        # long silences in 1 s passes, then a timer-read block over the return
        run_for(d - 2000, 5000 if d > 100000 else None)
        if random.random() < 0.5:
            b = random.randint(2000, 30000); note("block timer over return", b); block(b, True)
    elif x < 0.75:
        note("restart"); rx.restart(clk[0])
    elif x < 0.80:
        note("restart banner lost"); rx.restart(clk[0]); rx.buf = bytearray()
    elif x < 0.85:
        note("restart during silence"); d = random.randint(61000, 3 * 86400000)
        rx.silent_until = clk[0] + d
        run_for(d - 1000, 5000 if d > 100000 else None)
        rx.restart(clk[0] + 1000)
    elif x < 0.90:
        note("fixless"); rx.fixless_until = clk[0] + random.randint(1000, 600000)
    else:
        rx.ignore = random.random() < 0.5; rx.mute_cmd = random.random() < 0.2
        note("cmd ignore", rx.ignore, rx.mute_cmd)
    if bad[0] > 10: break
# quiet: a well-behaved receiver for 20 min
rx.ignore = rx.mute_cmd = False; rx.drop = rx.cut = rx.rmc_drop = 0
rx.fixless_until = 0; rx.silent_until = 0
run_for(20 * 60000)
s = r.st
ok = rx.hz == 5 and rx.gst == 1
if not ok:
    print("STUCK hz", rx.hz, "gst", rx.gst, "setup_ok", s["setup_ok"], "extra", s["extra"], "wait", s["setup_wait"], "at", s["setup_at"], "boot_at", s["boot_at"], "storm", s["storm_at"], "events", events[-5:])
    bad[0] += 1
print("seed", seed, "bad", bad[0], "ok", s["ok"], "err", s["err"], "boots", s["gps_boots"], rx.restarts, "diag", len(DIAG), "days", (clk[0] // 86400000))
