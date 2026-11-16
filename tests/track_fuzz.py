"""Power-loss / fault-injection fuzz of the track writer (flush, _close_file,
start_file, segment breaks, the stay's hold, a full flash): LittleFS's commit
on close emulated (an empty entry at "wb", a close that fails committing or
not), partial writes, MemoryError in the encoder, open and read, a power cut
at any open, write, close or remove (what was not committed is gone, the
logger starts again: log_reset). Every recorded point must be read back from
the files exactly once, in order, unless it was in RAM at a cut or dropped
on purpose (log["drop"]): no duplicate, no garbage point.

    python3 tests/track_fuzz.py [SEED[:CC] ...]   -> one line a seed, exit 1 on a fault

Runs the suite's harness (tests/test_gps_logger.py up to "# FLASH-CACHE:"),
CPython only. Not deployed."""
import os
import sys

import builtins
import errno
import random
import tempfile
import types
from collections import Counter

_HERE = os.path.dirname(os.path.abspath(__file__))
_TEST = os.path.join(_HERE, "test_gps_logger.py")
with open(_TEST, encoding="utf-8") as _f:
    _src = _f.read()
_H = {"__file__": _TEST, "__name__": "track_fuzz_harness"}
exec(compile(_src[:_src.index("\n# FLASH-CACHE:")], _TEST, "exec"), _H)
_H["_run_ended"].append(1)               # (the harness's summary: not a run)
gl, trk_line, trk2gpx = _H["gl"], _H["trk_line"], _H["trk2gpx"]

_SAVED = ("open", "os", "free", "flush", "_record", "_kb_part", "_still_hold", "_diag_write",
          "HIST_DIR")


import re  # noqa: E402
_TIME_END = re.compile(r",\d{8}T\d{6}Z\n$")   # a line that ends in its time
_NONE = object()                         # (gl.open: the builtin, no attribute)


class PL(BaseException):
    """A power cut."""


def run(seed, n_steps=3000, plp=0.01, memp=0.03, cc=0.5, statp=0.005):
    rnd = random.Random(seed)
    saved = {k: vars(gl).get(k, _NONE) for k in _SAVED}
    d = tempfile.mkdtemp()
    gl.LOG_DIR = d
    gl.HIST_DIR = d + "/hist"
    gl.log_reset()
    base = gl._dt_s("2026-10-18", "21:00:00")   # (past midnight: two UTC days)
    stat_eio = [0.0]
    hist_max = {}                        # day file -> the largest size seen
    hist_shrunk = []
    hist_gone = []                       # the newest day's file removed
    free_ = [10 ** 8]
    committed = {}
    fault = {"write": 0.0, "close": 0.0, "pl": 0.0}
    mem = [0.0]
    hold = [None]

    def maybe_pl():
        if rnd.random() < fault["pl"]:
            raise PL()

    class F(object):
        def __init__(self, path, mode):
            maybe_pl()
            if rnd.random() < mem[0]:
                raise MemoryError("open")
            self.path, self.pos = path, 0
            if "w" in mode:
                self.buf = bytearray()
                committed[path] = b""
            else:
                if path not in committed:
                    raise OSError(errno.ENOENT)
                self.buf = bytearray(committed[path])

        def write(self, b):
            b = bytes(b)
            maybe_pl()
            if rnd.random() < fault["write"]:
                self._put(b[:rnd.randrange(len(b) + 1)])
                raise OSError(errno.ENOSPC)
            self._put(b)
            return len(b)

        def _put(self, b):
            self.buf[self.pos:self.pos + len(b)] = b
            self.pos += len(b)

        def read(self, n=-1):
            if rnd.random() < mem[0]:
                raise MemoryError("read")
            if n < 0:
                n = len(self.buf) - self.pos
            r = bytes(self.buf[self.pos:self.pos + n])
            self.pos += len(r)
            return r

        def readinto(self, mv):
            r = self.read(len(mv))
            mv[:len(r)] = r
            return len(r)

        def seek(self, off, wh=0):
            self.pos = off if wh == 0 else (self.pos + off if wh == 1 else len(self.buf) + off)
            return self.pos

        def tell(self):
            return self.pos

        def close(self):
            maybe_pl()
            if rnd.random() < fault["close"]:
                if rnd.random() < cc:
                    committed[self.path] = bytes(self.buf)
                raise OSError(errno.EIO)
            committed[self.path] = bytes(self.buf)

        def __enter__(self):
            return self

        def __exit__(self, *a):
            self.close()

    def fopen(path, mode="r"):
        return F(path, mode) if path.startswith(d) else builtins.open(path, mode)

    def fstat(p):
        if p.startswith(d):
            if p not in committed:
                raise OSError(errno.ENOENT)
            if rnd.random() < stat_eio[0]:
                raise OSError(errno.EIO)     # (transient: not "not there")
            return (0, 0, 0, 0, 0, 0, len(committed[p]), 0, 0, 0)
        return os.stat(p)

    def frm(p):
        if p.startswith(d):
            maybe_pl()
            if p not in committed:
                raise OSError(errno.ENOENT)
            if p.startswith(gl.HIST_DIR + "/"):
                days = sorted(k[-12:-4] for k in committed if k.startswith(gl.HIST_DIR + "/"))
                today = gl._stamp_s(base + sec[0])[0][:10].replace("-", "")
                if days and p[-12:-4] == days[-1] and p[-12:-4] >= today:
                    hist_gone.append(p[-12:])    # (today's: no trim takes it)
            del committed[p]
            hist_max.pop(p, None)        # (an older day trimmed: no shrink)
            return
        os.remove(p)

    fos = types.SimpleNamespace(**{k: getattr(os, k) for k in dir(os) if not k.startswith("__")})
    fos.stat, fos.remove = fstat, frm
    fos.mkdir = lambda p: None
    fos.listdir = lambda p: ([k[len(p) + 1:] for k in committed
                              if k.rsplit("/", 1)[0] == p] if p.startswith(d) else os.listdir(p))
    dropped = set()
    kp0, fl0, rc0 = saved["_kb_part"], saved["flush"], saved["_record"]

    def kp(lines, k, seg):
        if rnd.random() < mem[0]:
            raise MemoryError("encode")
        return kp0(lines, k, seg)

    def fl(h=None, fx=None):
        d0, before = gl.log["drop"], list(gl.log["batch"])
        try:
            return fl0(h, fx)
        finally:
            if gl.log["drop"] > d0:
                dropped.update(before)

    def rc(s, pos, fx=None):
        d0, b0 = gl.log["drop"], gl.log["batch"][:1]
        rc0(s, pos, fx)
        if gl.log["drop"] > d0:
            dropped.update(b0)

    gl.open, gl.os, gl.free = fopen, fos, lambda: free_[0]
    gl.flush, gl._record, gl._kb_part = fl, rc, kp
    gl._still_hold = lambda: hold[0]
    gl._diag_write = lambda *a: None
    sec = [0]
    recorded = []
    lost_ram = set()
    gl.log["clock"] = (base, gl._stamp())

    def hist_check():
        for k, v in committed.items():
            if k.startswith(gl.HIST_DIR + "/"):
                if len(v) < hist_max.get(k, 0):
                    hist_shrunk.append((k[-12:], hist_max[k], len(v)))
                hist_max[k] = max(hist_max.get(k, 0), len(v))
    try:
        for step in range(n_steps):
            on = step < n_steps - 300
            fault["write"] = fault["close"] = 0.02 if on else 0
            fault["pl"] = plp if on else 0
            mem[0] = memp if on else 0
            stat_eio[0] = statp if on else 0
            free_[0] = 10 ** 8 if (not on or rnd.random() > 0.03) else 1000
            try:
                r = rnd.random()
                if r < 0.7:
                    sec[0] += rnd.choice([1, 1, 1, 2, 5, 70])
                    s_ = sec[0]
                    t = gl._stamp_s(base + s_)[0]       # (days on: no hour % 24)
                    line = trk_line(47.5 + rnd.randrange(-50, 50) * 1e-5 + s_ * 1e-6,
                                    19.0 + rnd.randrange(50) * 1e-5, t,
                                    ele=rnd.choice(["10.0", "11.5", ""]),
                                    hdop=rnd.choice(["", "1.2", "0.9"]))
                    if gl.log["file"] is None:
                        tc = line[-17:-1]
                        gl.start_file(tc[:8] + "_" + tc[9:15])
                    gl._record((t, line), (0, 0))
                    recorded.append(line)
                elif r < 0.75:
                    gl._seg_break()
                elif r < 0.95:
                    b = gl.log["batch"]
                    hold[0] = b[rnd.randrange(len(b))][-17:-1] if b and rnd.random() < 0.5 else None
                    if r < 0.9:
                        gl.flush(hold[0])
                    else:
                        gl._close_file()
                elif r < 0.96:
                    raise PL()
                elif r < 0.99:
                    # the day history of a closed track's points, as after its
                    # 2xx (_hist_batch): no exception out of it, no day file
                    # shrinks (a stat EIO made it a "wb" over the day)
                    gl.log["clock"] = (base + sec[0], gl._stamp())
                    tracks = [k for k in committed if k.endswith(".trb")
                              and not k.startswith(gl.HIST_DIR) and k != gl.log["file"]]
                    if tracks:
                        p_ = rnd.choice(sorted(tracks))
                        gl._hist_batch(p_, 0, len(committed[p_]), None)
                    hist_check()
            except MemoryError:          # run()'s crash branch: a flush, a reset
                try:
                    gl.flush(hold[0])
                except (MemoryError, PL):
                    pass
                lost_ram.update(gl.log["batch"])
                gl.log_reset()
                hold[0] = None
            except PL:
                lost_ram.update(gl.log["batch"])
                gl.log_reset()
                hold[0] = None
        fault.update(write=0, close=0, pl=0)
        mem[0], hold[0], free_[0] = 0, None, 10 ** 8
        for _ in range(5):
            gl.flush(None)
        got, garbage, order = [], [], True
        rs = set(recorded)
        hist_check()
        hist_garbage = 0
        hist_lines = []
        for p in sorted(committed):
            if not p.endswith(".trb"):
                continue
            if p.startswith(gl.HIST_DIR + "/"):
                # (an append a cut left short ends in HIST_SEP's 0xC0 bytes:
                # a record without its time, no point the page draws)
                hg = [x for x in trk2gpx.trb_lines(committed[p])
                      if x[:1] not in ("#", "-") and "," in x and x not in rs
                      and _TIME_END.search(x)]
                hist_garbage += len(hg)
                hist_lines.extend(hg[:2])
                continue
            # (a record cut by a power cut reads as a line without a comma,
            # "END", "47D": no point, the upload's _trk_point refuses it)
            pts = [x for x in trk2gpx.trb_lines(committed[p])
                   if x[:1] not in ("#", "-") and "," in x]
            garbage += [(p[-19:], x) for x in pts if x not in rs]
            pts = [x for x in pts if x in rs]
            order = order and pts == sorted(pts, key=lambda x: x[-17:])
            got += pts
        miss = Counter(recorded) - Counter(got)
        extra = sum((Counter(got) - Counter(recorded)).values())
        unexplained = sum(1 for x in miss.elements() if x not in lost_ram and x not in dropped)
        return {"seed": seed, "recorded": len(recorded), "got": len(got), "extra": extra,
                "unexplained": unexplained, "garbage": len(garbage), "order": order,
                "garbage_lines": garbage[:2],
                "hist_shrunk": hist_shrunk[:2] + hist_gone[:2], "hist_garbage": hist_garbage, "hist_lines": hist_lines[:2],
                "left": len(gl.log["batch"])}
    finally:
        for k, v in saved.items():
            if v is _NONE:
                delattr(gl, k)
            else:
                setattr(gl, k, v)
        gl.log_reset()


if __name__ == "__main__":
    bad = 0
    for a in sys.argv[1:] or ["1"]:
        seed, _, cc = a.partition(":")        # SEED or SEED:CC (the share of
        res = run(int(seed), cc=float(cc) if cc else 0.5)   # failed closes that commit)
        ok = (res["extra"] == 0 and res["unexplained"] == 0 and res["garbage"] == 0
              and res["order"] and res["left"] == 0 and not res["hist_shrunk"]
              and res["hist_garbage"] == 0)
        bad += not ok
        print("%s %s" % ("ok" if ok else "FAULT", res))
    sys.exit(1 if bad else 0)
