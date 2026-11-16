"""The OTA's whole life under a power cut at each flash step: a bootloader
model (otadata: the boot slot, each image NEW / PENDING / VALID / ABORTED),
gps_boot.unshadow() (and its cause note) before ota_boot at every boot, a
shadowing /gps_logger.mpy on flash, and one cut at the n-th flash
operation counted over the whole of it (the install, every boot's gps_boot
and ota_boot, the pending run, the confirmation, the rollback); after the
cut it runs on without one until it is stable. At the end: the running
firmware has its own module and page, a new one is confirmed, nothing good
is marked bad.

    python3 tests/ota_cutsim.py [plain|nowifi|wdt|crash ...]   -> exit 1 on a fault

A cut lands before each open for writing, right after it (the empty file
LittleFS leaves) and at each write, rename, remove and set_boot; a run
that never settles in 12 boots is a fault too. crash: the new build
crashes at its first run.

Runs the suite's harness (tests/test_gps_logger.py up to "# FLASH-CACHE:"),
CPython only. Not deployed."""
import builtins
import json
import os
import sys
import tempfile
import types

_HERE = os.path.dirname(os.path.abspath(__file__))
_TEST = os.path.join(_HERE, "test_gps_logger.py")
with open(_TEST, encoding="utf-8") as _f:
    _src = _f.read()
_H = {"__file__": _TEST, "__name__": "ota_cutsim_harness"}
exec(compile(_src[:_src.index("\n# FLASH-CACHE:")], _TEST, "exec"), _H)
_H["_run_ended"].append(1)               # (the harness's summary: not a run)
gl, _gboot, _reset_cause, _clock = _H["gl"], _H["_gboot"], _H["_reset_cause"], _H["_clock"]
import hashlib as _hl  # noqa: E402
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
_ota_saved2 = (gl.OTA_MARK, gl.OTA_SHADOW)
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
              getattr(gl, "OTA_DL", None) or os.path.join(_ota_dir, "-"),
              gl.OTA_MARK or os.path.join(_ota_dir, "-"),
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



_os = os
MODE = "plain"
class Cut(BaseException): pass
class Reset(BaseException): pass
cut = {"n": None, "k": 0, "log": [], "ops": []}
def hit(what):
    cut["k"] += 1
    cut["ops"].append(what)
    if cut["n"] is not None and cut["k"] == cut["n"]:
        cut["log"].append(what)
        cut["n"] = None
        raise Cut()
_ropen = builtins.open
def xopen(p, mode="r", *a, **k):
    if any(m in mode for m in "wa+"):
        hit("open %s %s" % (os.path.basename(str(p)), mode))
    f = _ropen(p, mode, *a, **k)
    if "w" in mode:
        # (LittleFS: the "w" open made an empty file; a cut before its close
        # leaves it so: an empty mark, note or count)
        try:
            hit("opened %s %s" % (os.path.basename(str(p)), mode))
        except Cut:
            f.close()
            raise
    return f
class OsP(object):
    def __getattr__(s, n): return getattr(_os, n)
    def rename(s, a, b): hit("rename %s" % os.path.basename(a)); return _os.rename(a, b)
    def remove(s, a): hit("remove %s" % os.path.basename(a)); return _os.remove(a)
gl.open = xopen
gl.os = OsP()
_gboot.open = xopen
_gboot_os = OsP()
sys.modules["os_cut"] = _gboot_os
# gps_boot does "import os" inside functions: give it ours via builtins hook
_real_import = builtins.__import__
def _imp(name, g=None, l=None, fromlist=(), level=0):
    if name == "os" and g is not None and g.get("__name__") == "gps_boot":
        return _gboot_os
    return _real_import(name, g, l, fromlist, level)
builtins.__import__ = _imp
_gboot.OTA_PENDING = gl.OTA_PENDING
_gboot.OTA_MARK = gl.OTA_MARK
_gboot.OTA_SHADOW = gl.OTA_SHADOW
frozen = types.ModuleType("gps_frozen")
sys.modules["gps_frozen"] = frozen

# --- bootloader / otadata model
ot = {"boot": "ota_0", "state": {"ota_0": "VALID", "ota_1": None}, "running": "ota_0"}
SLOT_FW = {"ota_0": 100, "ota_1": 200}
def _set_boot(self):
    hit("set_boot")
    ot["boot"] = self.name
    ot["state"][self.name] = "NEW"
_FakePart.set_boot = _set_boot
_wb = _FakePart.writeblocks
def wb(self, blk, buf): hit("write"); _wb(self, blk, buf)
_FakePart.writeblocks = wb
byname = {"ota_0": _slots[0], "ota_1": _slots[1]}
class P(object):
    RUNNING = 1
    BOOT = 0
    def __new__(cls, which):
        if which == cls.BOOT:
            return byname[ot["boot"]]
        return byname[ot["running"]]
    @staticmethod
    def mark_app_valid_cancel_rollback():
        hit("mark_valid")
        r = ot["running"]
        if ot["state"][r] == "PENDING":
            ot["state"][r] = "VALID"
_FakeEsp32.Partition = P
def _gnu(self):
    return _slots[1] if self is _slots[0] else _slots[0]
_FakePart.get_next_update = _gnu
def bootloader():
    b = ot["boot"]
    s = ot["state"][b]
    if s == "NEW":
        ot["state"][b] = "PENDING"; ot["running"] = b
    elif s == "PENDING":
        ot["state"][b] = "ABORTED"
        other = "ota_0" if b == "ota_1" else "ota_1"
        ot["boot"] = other; ot["running"] = other
    else:
        ot["running"] = b
    return SLOT_FW[ot["running"]]
def _reset():
    raise Reset()
gl.machine.reset = _reset

gl.HTTP_GET = _ota_get
gl.OTA_URL, gl.OTA_KEY = "http://ota.lan/gps/manifest.json", b"secret"
page_p = os.path.join(_ota_dir, "web.html.gz")
mod_p = gl.OTA_SHADOW[0]
def ram_reset():
    gl.ota.clear()
    gl.ota.update(at=None, state="idle", boot_at=gl._stamp(), pend=None, pend_at=None, wifi_seen=False)
    gl.log_reset()
    gl.wifi_reset()

def one_boot(cause):
    """bootloader, gps_boot.unshadow, ota_boot; -> ('reset'|'run', fw, imported-module-present)"""
    fw = bootloader()
    gl.FW_BUILD, gl.FW_COMMIT = fw, "abc%04d" % fw
    frozen.BUILD = fw
    _reset_cause[0] = cause
    ram_reset()
    _gboot.unshadow()
    shadowed = os.path.exists(mod_p)
    if MODE == "wdt" and fw == 100 and os.path.exists(gl.OTA_PENDING) and not wdt_once["done"]:
        wdt_once["done"] = True
        return "wdt", fw, shadowed
    try:
        gl.ota_boot()
    except Reset:
        return "reset", fw, shadowed
    return "run", fw, shadowed

def run_pending(wifi_ok, ticks=200):
    gl.wifi["state"] = "connected" if wifi_ok else "not found"
    if MODE == "crash" and gl.FW_BUILD == 200:
        # the new build crashes: run()'s crash branch, then its reset
        gl._ota_mark("crash")
        return "reset"
    for i in range(ticks):
        if gl.ota["pend"] is None:
            return "stable"
        _clock[0] += 5000
        try:
            gl.ota_tick(_clock[0])
        except Reset:
            return "reset"
    return "stuck"

def lifecycle(n, wifi_ok=True, extra=None):
    """-> (trace, final summary)"""
    _ota_reset()
    ot.update(boot="ota_0", running="ota_0"); ot["state"] = {"ota_0": "VALID", "ota_1": None}
    for p in (gl.OTA_SHADOW[0] + ".ota-old", mod_p, gl.OTA_TRIES):
        if os.path.exists(p): os.remove(p)
    _ropen(page_p, "wb").write(b"old page")
    _ropen(mod_p, "wb").write(b"old module")
    _serve(_manifest(200))
    gl.FW_BUILD, gl.FW_COMMIT = 100, "abc0100"; frozen.BUILD = 100
    ram_reset()
    cut.update(n=n, k=0, log=[], ops=[])
    wdt_once["done"] = False
    trace = []
    cause = None
    try:
        gl._ota_check()
        trace.append("check returned (no reset)")
        return trace, None
    except Reset:
        cause = 2; trace.append("install->reset")
    except Cut:
        cause = 1; trace.append("CUT@%s" % cut["log"][-1])
    for b in range(12):
        try:
            what, fw, shadowed = one_boot(cause)
        except Cut:
            cause = 1; trace.append("boot CUT@%s" % cut["log"][-1]); continue
        trace.append("boot%d%s" % (fw, "+mpy" if shadowed else ""))
        if what == "wdt":
            cause = 3; trace.append("WDT-before-ota_boot"); continue
        if what == "reset":
            cause = 2; trace.append("->reset"); continue
        if gl.ota["pend"] is None:
            break
        try:
            r = run_pending(wifi_ok)
        except Cut:
            cause = 1; trace.append("pend CUT@%s" % cut["log"][-1]); continue
        trace.append(r)
        if r == "reset":
            cause = 2; continue
        break
    else:
        trace.append("loop")             # (12 boots and never stable)
    return trace, summary(fw, shadowed)

def summary(fw, shadowed):
    ex = sorted(f for f in _os.listdir(_ota_dir) if f not in ("web.html.gz",))
    page = _ropen(page_p, "rb").read()[:8] if os.path.exists(page_p) else None
    bad = []
    if fw == 200 and shadowed: bad.append("OLD MODULE SHADOWS NEW FW")
    if fw == 100 and not shadowed: bad.append("old fw lost its module")
    if fw == 200 and ot["state"]["ota_1"] != "VALID": bad.append("200 running unconfirmed: " + str(ot["state"]))
    if fw == 200 and page != _page[:8]: bad.append("new fw old page")
    if fw == 100 and page != b"old page": bad.append("old fw page " + repr(page))
    if gl._ota_bad(): bad.append("BAD %s" % gl._ota_bad())
    return "fw%d st=%s tries=%s files=%s %s" % (fw, gl.ota["state"], gl._ota_tries(), ex, " ".join(bad))

wdt_once = {"done": False}
gl.OTA_DL = os.path.join(_ota_dir, "ota.dl")


def run_mode(mode):
    """Every cut point of one mode -> the faults (cut point, trace, summary)."""
    global MODE
    MODE = mode
    faults = []
    for n in range(1, 120):
        trace, s = lifecycle(n, mode != "nowifi")
        if not cut["log"]:
            return faults, n             # (past the last flash step: done)
        bads = ("SHADOWS", "lost its", "unconfirmed", "old page", "page b")
        if mode == "crash":
            # (a crashing build: the old firmware back with its own module and
            # page; bad, or a try where a cut took the crash mark)
            fw_ok = s is not None and s.startswith("fw100") and not any(w in s for w in bads)
        else:
            fw_ok = s is not None and not any(w in s for w in bads + ("BAD",))
        if not fw_ok or "stuck" in trace or "loop" in trace:
            faults.append((n, " ".join(trace), s))
    return faults, 120


if __name__ == "__main__":
    bad = 0
    for mode in sys.argv[1:] or ["plain", "nowifi", "wdt", "crash"]:
        faults, n = run_mode(mode)
        print("%s %s: %d cut points, %d faults" % ("ok" if not faults else "FAULT", mode, n - 1,
                                                  len(faults)))
        for f in faults[:5]:
            print("  ", f)
        bad += bool(faults)
    sys.exit(1 if bad else 0)
