# The logger's start (start(): main.py calls it) and what it needs around
# it on a frozen firmware (tools/build_firmware.sh freezes this with
# gps_logger): frozen, its bytecode runs from flash, and an OTA brings a new
# one, while main.py (a file, compiled on the board at every boot) changes
# only with deploy.sh. In main.py this took ~0.7 KB of the Python heap for
# the whole run and ~5 KB more while main.py compiled. Without it (a plain
# firmware: deploy.sh's .mpy), or with an older one without start() (the
# old firmware after a rollback), main.py starts the logger itself.
import machine

OTA_PENDING = "/ota.pending"             # (gps_logger's OTA_PENDING,
OTA_MARK = "/ota.mark"                   # OTA_MARK and OTA_SHADOW)
OTA_SHADOW = ("/gps_logger.mpy", "/gps_logger.py",
              "/gps_clock.mpy", "/gps_clock.py",
              "/gps_receiver.mpy", "/gps_receiver.py",
              "/gps_tft.mpy", "/gps_tft.py",
              "/gps_tft_font.mpy", "/gps_tft_font.py")
SKIP_PIN = 7                             # (main.py's: GPIO7 low at reset
BOOT_FAIL_MAX = 5                        # skips the logger; the counter in
BOOT_FAIL_FILE = "/boot_fail"            # RTC memory, else this file;
BOOT_RETRY_MS = 1800000                  # gps_logger clears it after
DIAG_FILE = "/diag.log"                  # BOOT_OK_MS; gps_logger's DIAG_FILE)

_retry = None                            # the BOOT_RETRY_MS timer (REPL:
                                         # gps_boot._retry.deinit() stops it)

def _note():
    """(the build OTA_PENDING names or None, the running firmware's BUILD)."""
    try:
        from gps_frozen import BUILD
    except ImportError:
        BUILD = 0
    try:
        with open(OTA_PENDING) as f:
            note = int(f.read().split(None, 1)[0])
    except Exception:
        note = None
    return note, BUILD

def unshadow():
    """An OTA install moves /gps_logger.mpy (.py) aside as <name>.ota-old so
    the new firmware's frozen logger is imported, not it. Unless that build
    runs pending (the note names the running BUILD), it comes back here,
    before the import: after a rollback, and after the health reset (the
    note gone) the old firmware gets the module it ran. One deployed since
    stays (the .ota-old goes). A failing remove leaves the .ota-old: only
    a module that is not there is replaced by it. A note of another build
    (the old firmware back after a rollback) gets this boot's reset cause
    (_cause) first."""
    note, BUILD = _note()
    if note is not None and note == BUILD:
        _ran()
        return
    if note is not None:
        _cause()
    import os
    for p in OTA_SHADOW:
        try:
            os.stat(p + ".ota-old")
        except OSError:
            continue
        try:
            os.stat(p)
        except OSError:
            try:
                os.rename(p + ".ota-old", p)
            except OSError:
                pass
            continue
        try:
            os.remove(p + ".ota-old")
        except OSError:
            pass

def _mark():
    try:
        with open(OTA_MARK) as f:
            return f.read().strip()
    except Exception:
        return None

def _ran():
    """The new build boots: the install's "staged" becomes "ran" before
    its import. A build that died before its own ota_boot (a hang, a
    panic) left "staged", and the old firmware took that for an install
    cut before set_boot: no try, no verdict, installed again for good.
    Never raises."""
    if _mark() != "staged":
        return
    try:
        with open(OTA_MARK, "w") as f:
            f.write("ran")
    except Exception:
        pass

def _cause():
    """The old firmware's first boot after a rollback: its reset cause in
    OTA_MARK ("cause <n>") for ota_boot, unless a mark that says more is
    there (the new build's crash or stop, ota_boot's boot, an install's
    staged). "ran" (the new build got as far as its import) and "valid"
    (a power cut while it was confirmed: it never completed) are
    overwritten: "valid" left there, a watchdog reset of the old firmware
    before its ota_boot was read as the new build's, bad for good.
    ota_boot read the cause of its own boot: a reset of the old firmware
    before it ran (a Ctrl-C or a stall, then the watchdog) was taken for
    the new build's, a watchdog verdict on a good build. Never raises."""
    m = _mark()
    if m is not None and m not in ("ran", "valid"):
        return
    try:
        c = machine.reset_cause()
        with open(OTA_MARK, "w") as f:
            f.write("cause %d" % c)
    except Exception:
        pass

def crash():
    """main.py caught the logger failing (its import, or run() before its
    own crash branch): with an OTA note naming the running build there,
    "crash" in OTA_MARK, as run()'s crash branch writes, so the old firmware
    after the rollback blames the build. Without it a pending image that
    cannot even import counted as an external reset: a try, installed again
    OTA_TRIES_MAX times. Not for a note of another build (the old firmware,
    running after a rollback, failed: the new build's note is not cleared
    before ota_boot) or one that cannot be read. Never raises."""
    note, BUILD = _note()
    if note is None or note != BUILD or _mark() == "valid":
        return                           # (valid: confirmed, not a crash's)
    try:
        with open(OTA_MARK, "w") as f:
            f.write("crash")
    except Exception:
        pass

def safe(n, retry_ms, again):
    """BOOT_FAIL_MAX failed starts: the REPL (mpremote, no WDT), and after
    retry_ms one more start (again(): the count set back by one, then a
    reset; it failing again is another retry_ms here). It sat in the REPL
    for good, not logging, until someone came with a cable. A virtual
    timer (-1, esp_timer): hardware Timer(0) is the logger's Ctrl-C
    keep-alive, one started from the REPL took it over."""
    global _retry
    print("boot fail", n, "- REPL; one more start in", retry_ms // 60000,
          "min (gps_boot._retry.deinit() stops it)")
    try:
        _retry = machine.Timer(-1)
        _retry.init(period=retry_ms, mode=machine.Timer.ONE_SHOT,
                    callback=lambda t: again())
    except Exception as e:
        print("no retry timer", e)

# --- the start (main.py's, here so that an OTA brings it) -------------------

def _count(raw):
    """The boot-fail count in RTC memory or BOOT_FAIL_FILE: its leading
    digits, 0 for none."""
    if not raw:
        return 0
    if isinstance(raw, bytes):
        raw = raw.split(b"\x00", 1)[0].split(b"\n", 1)[0].decode()
    n = 0
    for c in str(raw).strip():
        if c < "0" or c > "9":
            break
        n = n * 10 + (ord(c) - 48)
    return n

def fail_get():
    try:
        return _count(machine.RTC().memory())
    except Exception:
        try:
            with open(BOOT_FAIL_FILE) as f:
                return _count(f.read())
        except Exception:
            return 0

def fail_set(n):
    """-> whether the count was stored (RTC memory, else the file)."""
    try:
        machine.RTC().memory(b"%d" % n)
        return True
    except Exception:
        pass
    try:
        with open(BOOT_FAIL_FILE, "w") as f:
            f.write("%d" % n)
        return True
    except Exception:
        return False

def diag(line):
    """One short line appended to DIAG_FILE (gps_logger rotates it). Never
    raises."""
    try:
        with open(DIAG_FILE, "a") as f:
            f.write(line[:200] + "\n")
    except Exception:
        pass

def _again():
    fail_set(BOOT_FAIL_MAX - 1)
    machine.reset()

def _safe(n, why):
    diag("crash boot fail %d%s: REPL, one more start in %d min" % (
        n, why, BOOT_RETRY_MS // 60000))
    safe(n, BOOT_RETRY_MS, _again)

def _failed(e):
    """The logger's import or run() raised (before run()'s own crash
    branch): printed, a diag line, counted, the build's crash while an OTA
    is pending. -> True: reset after the delay; False: the REPL. A count
    that cannot be stored (no RTC memory, no file) goes to the REPL with
    its retry too: it reset every 5 s for good."""
    import sys
    import time
    try:
        sys.print_exception(e)
    except Exception:
        print("boot", repr(e))
    n = fail_get() + 1
    kept = fail_set(n)
    crash()                              # (an OTA pending: the build's fault)
    diag("crash boot %d %s" % (n, repr(e)[:120]))
    if n >= BOOT_FAIL_MAX or not kept:
        _safe(n, "" if kept else " (count not stored)")
        return False
    time.sleep(min(5 * n, 30))
    return True

def start():
    """main.py's start: the boot-bypass pin, the boot-fail counter (a WDT
    reset counts, it never reaches an except), BOOT_FAIL_MAX failed starts
    to the REPL with a retry, unshadow() before the import, the logger's
    run(). The counter is not cleared here: gps_logger does after BOOT_OK_MS
    of a healthy loop (wifi_start or run() can still crash)."""
    if not machine.Pin(SKIP_PIN, machine.Pin.IN, machine.Pin.PULL_UP).value():
        return
    try:
        if machine.reset_cause() == machine.WDT_RESET:
            fail_set(fail_get() + 1)
    except Exception:
        pass
    n = fail_get()
    if n >= BOOT_FAIL_MAX:
        _safe(n, "")
        return
    try:
        unshadow()                       # before the import
        import gps_logger
        gps_logger.run()
    except KeyboardInterrupt:
        raise
    except Exception as e:
        if _failed(e):
            machine.reset()
