# Development

[← README](../README.md)

GitHub Actions (`.github/workflows/ci.yml`) runs on pushes to `master` and on
every pull request:

- `python3 tests/test_gps_logger.py`: the CPython harness with mocked
  `machine`/`network`/UART (exits 1 on any FAIL), plus
  `tests/mpy_unix_check.py` and `tests/mpy_unix_logger.py` (imports
  `gps_logger.py` with stub `machine`/`network` modules and runs its pure
  paths) on a cached MicroPython 1.29 unix build, and the smoke again on a
  single-precision build (`MICROPY_FLOAT_IMPL_FLOAT`, like the board: the
  departure ring must rebuild the lines `_sample_line` writes); the smoke
  also runs under CPython. It also runs:
  - `tests/web_loopback.py` on the unix build and CPython: the status page
    over real loopback sockets (a normal request, a foreign `Host`, an
    empty, an oversized, a stalled and a trickling request, a client
    resetting mid-reply, a reply past the request deadline, the gzipped
    page for a client that accepts it, and for one that does not the page
    inflated from the `.gz` alone, byte for byte, its live heap under
    10 KB on the unix build);
  - `node tests/web_page_test.js`: `web.html`'s view on generated
    `status.json`, and the whole page script (`tick`, `paint`, `placeMap`,
    `stale`) against a stub document, and the same on the page as
    `tools/web_min.py` strips it (it must print the same). With `CI=true`
    (GitHub Actions) a missing `node` is a FAIL, not a SKIP;
  - `tests/rcv_sim.py` on the single-precision build: a stateful UC6580
    and the loop on a fake clock (setup, restarts with and without a
    banner, silences of days, blocks, lost sentences): no fix with a day
    it was not sent on, the setup back at the end;
  - `tests/track_fuzz.py`: the track writer under power cuts, partial
    writes, failed closes, transient stat errors and MemoryErrors
    (LittleFS's commit on close), over two UTC days, with the day history
    of closed tracks: every point read back once and in order, no day file
    shrinking but by its trim;
  - `tests/ota_cutsim.py`: the OTA's whole life with a bootloader model and
    a power cut at each flash step (WiFi, none, the old firmware's
    watchdog before its ota_boot): the running firmware has its own module
    and page, a new one is confirmed, no good build is bad;
  - `deploy.sh` against a fake board (a stand-in `mpremote` that runs the
    board-side code under CPython): copy, check and rename, too little
    flash, a cut copy, frozen firmware (stale, `FORCE`, `FORCE_MPY`, no
    clear answer, a marker that is not plain words, a relative `VENV`), an OTA
    pending (`FORCE_PENDING`);
  - `tools/build_firmware.sh` with a fake ESP-IDF, MicroPython tree,
    `esptool.py`, `mpremote`, `scp` and `ssh`: the `gps_frozen` marker,
    `--flash` touching nothing on the board before a successful write,
    `--migrate`'s flash size check and private backup, `--publish` moving each
    file in from a `.tmp` and refusing a build not newer than the published
    one, a key file readable by everyone, and an `OTA_DEST` that is not a
    plain path.
  The page checks run on Node 24.22 (`actions/setup-node`, pinned; the
  stripped page's AST check reaches node's own acorn). The page's
  layout check (headless Chrome at 375 and 800 px) runs where `google-chrome`
  or `chromium` is on the `PATH`, else it is skipped; the workflow installs
  neither (the ubuntu-24.04 runner image has `google-chrome`).
  `TFT-FONT-REGEN` runs `tools/make_tft_font.py` and compares its output
  with the committed `gps_tft_font.py` (the workflow installs Pillow 11.0.0;
  elsewhere it is skipped without that Pillow). A check that cannot run
  prints `SKIP` and is counted in the summary (`N passed, N failed, N
  skipped`, then the skipped and the failed ones); with `CI=true` (GitHub
  Actions) a skip is a FAIL, as the workflow installs all the suite needs
  (`CI_SKIP_OK` lists the exceptions, none now). The summary is printed also
  when an uncaught exception ends the run early (`RUN ENDED EARLY`). The
  suite removes every temporary directory it made when it ends
  (`KEEP_TMP=1` keeps them to look at after a failure).
- `mpy-cross` 1.29 for `xtensawin` on every frozen module (`gps_logger.py`,
  `gps_clock.py`, `gps_receiver.py`, `gps_boot.py`, `gps_tft.py`, `gps_tft_font.py`) and `main.py`. The firmware
  itself (ESP-IDF, `boards/GPS_LOGGER_S3`) is not built in CI: a board or
  freeze-manifest breakage shows at the next local `tools/build_firmware.sh`.
- Every action pinned by commit SHA, `pyflakes`, `mpy-cross` and Pillow by
  exact version; the MicroPython cache key includes the workflow's hash (a
  changed build recipe builds anew).
- `pyflakes` on every Python file: `gps_logger.py`, `gps_clock.py`, `gps_receiver.py`,
  `gps_boot.py`, `gps_tft.py`, `gps_tft_font.py`, `main.py`, `tools/` and `tests/`.
- `shellcheck deploy.sh tools/build_firmware.sh`.

The same locally:

```bash
.venv/bin/pip install "pyflakes==4.1.0" "mpy-cross==1.29.0.post2" "pillow==11.0.0"
.venv/bin/python -m pyflakes gps_logger.py gps_clock.py gps_receiver.py gps_boot.py gps_tft.py gps_tft_font.py main.py tools tests
shellcheck deploy.sh tools/build_firmware.sh
for m in gps_logger gps_clock gps_receiver gps_boot gps_tft gps_tft_font main; do .venv/bin/python -m mpy_cross -O3 -march=xtensawin -o /tmp/$m.mpy $m.py; done
MICROPYTHON=/path/to/micropython MICROPYTHON_F32=/path/to/micropython-f32 \
    python3 tests/test_gps_logger.py
```

The single-precision unix build: `make -C ports/unix
BUILD=build-f32 PROG=micropython-f32
CFLAGS_EXTRA=-DMICROPY_FLOAT_IMPL=MICROPY_FLOAT_IMPL_FLOAT` in a
MicroPython v1.29.0 tree.

`tests/device_bench.py` is an optional on-device timing probe; nothing under
`tests/` is copied to the board.
