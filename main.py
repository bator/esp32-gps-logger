from machine import Pin, RTC
import machine, time

# Hold GPIO25 at reset to skip the logger (REPL + mpremote, no WDT).
BOOT_FAIL_MAX = 5
BOOT_FAIL_FILE = "/boot_fail"

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

def _boot_handle_error():
    """Record a boot/import/run failure. True → sleep + caller should reset."""
    n = _boot_fail_get() + 1
    _boot_fail_set(n)
    if n >= BOOT_FAIL_MAX:
        print("boot fail", n, "- REPL")
        return False
    time.sleep(_boot_fail_delay(n))
    return True

def _boot_note_wdt():
    """A WDT reset never hits the Python except path; count it here."""
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
        print("boot fail", n, "- REPL")
        return
    try:
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

# Boot bypass: hold this pin low at reset to skip the logger (REPL).
# GPIO25 is the classic ESP32 button. None, or a pin the chip does not have
# (GPIO25 on the ESP32-C6, which MicroPython enables only up to GPIO23), means
# no bypass: the logger starts.
BYPASS_PIN = 25

def _bypass_level():
    if BYPASS_PIN is None:
        return 1
    try:
        return Pin(BYPASS_PIN, Pin.IN, Pin.PULL_UP).value()
    except ValueError:
        return 1

# MicroPython runs this file as __main__. Tests import it under another name
# so the auto-start (and a possible reset loop) does not run in the harness.
if __name__ == "__main__":
    if _bypass_level():
        _boot_start()
