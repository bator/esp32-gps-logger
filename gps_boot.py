# What main.py does around the logger's start that only a frozen firmware
# (tools/build_firmware.sh freezes this with gps_logger) needs: frozen, its
# bytecode runs from flash. In main.py (compiled on the board at every boot)
# it took ~0.7 KB of the Python heap for the whole run and ~5 KB more while
# main.py compiled. Without it (a plain firmware: deploy.sh's .mpy) main.py
# starts the logger as before: no OTA there, and the REPL after
# BOOT_FAIL_MAX is for good.
import machine

OTA_PENDING = "/ota.pending"             # (gps_logger's OTA_PENDING,
OTA_MARK = "/ota.mark"                   # OTA_MARK and OTA_SHADOW)
OTA_SHADOW = ("/gps_logger.mpy", "/gps_logger.py",
              "/gps_clock.mpy", "/gps_clock.py",
              "/gps_receiver.mpy", "/gps_receiver.py",
              "/gps_lora.mpy", "/gps_lora.py")

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
    a module that is not there is replaced by it."""
    note, BUILD = _note()
    if note is not None and note == BUILD:
        return
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
    if note is None or note != BUILD:
        return
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
