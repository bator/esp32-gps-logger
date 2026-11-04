from machine import Pin, RTC
import machine, sys, time

# Hold GPIO25 (GPIO10 on the ESP32-C6, which has no GPIO25; GPIO7 on the Heltec
# Wireless Tracker, whose button GPIO0 is the download mode) at reset to skip
# the logger (REPL + mpremote, no WDT).
_M = getattr(sys.implementation, "_machine", "")
SKIP_PIN = 10 if "ESP32C6" in _M else 7 if "Heltec" in _M else 25
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
    raw = ("%d" % n).encode()
    try:
        RTC().memory(raw)
        return
    except Exception:
        pass
    try:
        f = open(BOOT_FAIL_FILE, "w")
        f.write("%d" % n)
        f.close()
    except Exception:
        pass

def _boot_fail_delay(n):
    d = 5 * n
    if d > 30:
        d = 30
    return d

def _boot_retry():
    _boot_fail_set(BOOT_FAIL_MAX - 1)
    machine.reset()

def _boot_safe(n):
    """BOOT_FAIL_MAX failed starts: the REPL, and on a frozen firmware one
    more start after BOOT_RETRY_MS (gps_boot.safe)."""
    if gps_boot is None:
        print("boot fail", n, "- REPL")
    else:
        gps_boot.safe(n, BOOT_RETRY_MS, _boot_retry)

def _boot_handle_error():
    """Record a boot/import/run failure. True → sleep + caller should reset."""
    n = _boot_fail_get() + 1
    _boot_fail_set(n)
    if gps_boot is not None:
        gps_boot.crash()                 # (an OTA pending: the build's fault)
    if n >= BOOT_FAIL_MAX:
        _boot_safe(n)
        return False
    time.sleep(_boot_fail_delay(n))
    return True

def _boot_note_wdt():
    """A WDT reset never hits the Python except path; count it here.
    A panic is not counted: MicroPython 1.29 reports it as HARD_RESET, the
    same as machine.reset() (gps_logger's own crash reset, which counted
    already, its TLS memory reboot, mpremote reset)."""
    try:
        if machine.reset_cause() == machine.WDT_RESET:
            n = _boot_fail_get() + 1
            _boot_fail_set(n)
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
        # Do not clear the counter here: wifi_start / run() can still crash
        # (e.g. a missing str method). gps_logger clears it after BOOT_OK_MS
        # of a healthy _loop.
        gps_logger.run()
    except KeyboardInterrupt:
        raise
    except Exception:
        if _boot_handle_error():
            machine.reset()

# MicroPython runs this file as __main__. Tests import it under another name
# so the auto-start (and a possible reset loop) does not run in the harness.
if __name__ == "__main__":
    if Pin(SKIP_PIN, Pin.IN, Pin.PULL_UP).value():
        _boot_start()
