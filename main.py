from machine import Pin, RTC
import machine, sys, time

# The start is gps_boot.start() (frozen: an OTA updates it, this file only
# deploy.sh); below it is for a plain firmware and an older gps_boot without
# start() (after a rollback). GPIO7 low at reset skips the logger (REPL, no
# WDT; the board's button, GPIO0, is the download mode).
SKIP_PIN = 7
BOOT_FAIL_MAX = 5
BOOT_FAIL_FILE = "/boot_fail"
BOOT_RETRY_MS = 1800000                  # in the REPL after BOOT_FAIL_MAX:
                                         # one more start after this
try:
    import gps_boot                      # frozen with the logger (OTA, the
except ImportError:                      # retry): its code not in the heap
    gps_boot = None

def _parse_count(raw):
    if not raw:
        return 0
    if isinstance(raw, bytes):
        raw = raw.split(b"\x00", 1)[0].split(b"\n", 1)[0].decode()
    s = str(raw).strip()
    n = 0
    for c in s:
        if c < "0" or c > "9":
            break
        n = n * 10 + (ord(c) - 48)
    return n

def _boot_fail_get():
    try:
        return _parse_count(RTC().memory())
    except Exception:
        try:
            with open(BOOT_FAIL_FILE) as f:
                return _parse_count(f.read())
        except Exception:
            return 0

def _boot_fail_set(n):
    """-> whether the count was stored."""
    try:
        RTC().memory(("%d" % n).encode())
        return True
    except Exception:
        pass
    try:
        with open(BOOT_FAIL_FILE, "w") as f:
            f.write("%d" % n)
        return True
    except Exception:
        return False

def _boot_fail_delay(n):
    d = 5 * n
    if d > 30:
        d = 30
    return d

DIAG_FILE = getattr(gps_boot, "DIAG_FILE", "/diag.log")  # (gps_logger's)

def _diag(line):
    try:
        with open(DIAG_FILE, "a") as f:
            f.write(line[:200] + "\n")
    except Exception:
        pass

def _boot_retry():
    _boot_fail_set(BOOT_FAIL_MAX - 1)
    machine.reset()

def _boot_safe(n):
    """BOOT_FAIL_MAX failed starts: the REPL, and on a frozen firmware one
    more start after BOOT_RETRY_MS (gps_boot.safe)."""
    _diag("crash boot fail %d: REPL" % n)
    if gps_boot is None:
        print("boot fail", n, "- REPL")
    else:
        gps_boot.safe(n, BOOT_RETRY_MS, _boot_retry)

def _boot_handle_error():
    """Count a failed start. True: slept, reset. A count not stored: the
    REPL and its retry (no reset every 5 s for good)."""
    n = _boot_fail_get() + 1
    kept = _boot_fail_set(n)
    if gps_boot is not None:
        gps_boot.crash()                 # (an OTA pending: the build's fault)
    if n >= BOOT_FAIL_MAX or not kept:
        _boot_safe(n)
        return False
    time.sleep(_boot_fail_delay(n))
    return True

def _boot_note_wdt():
    """A WDT reset never hits the except path: counted here (a panic reads
    as HARD_RESET, as machine.reset(): not counted)."""
    try:
        if machine.reset_cause() == machine.WDT_RESET:
            _boot_fail_set(_boot_fail_get() + 1)
    except Exception:
        pass

def _boot_start():
    _boot_note_wdt()
    n = _boot_fail_get()
    if n >= BOOT_FAIL_MAX:
        _boot_safe(n)
        return
    try:
        if gps_boot is not None:
            gps_boot.unshadow()          # before the import
        import gps_logger
        gps_logger.run()                 # (it clears the count: BOOT_OK_MS)
    except KeyboardInterrupt:
        raise
    except Exception as e:
        if hasattr(sys, "print_exception"):
            sys.print_exception(e)
        _diag("crash boot %s" % repr(e)[:120])
        if _boot_handle_error():
            machine.reset()

# MicroPython runs this file as __main__. Tests import it under another name
# so the auto-start (and a possible reset loop) does not run in the harness.
if __name__ == "__main__":
    if hasattr(gps_boot, "start"):
        gps_boot.start()
    elif Pin(SKIP_PIN, Pin.IN, Pin.PULL_UP).value():
        _boot_start()
